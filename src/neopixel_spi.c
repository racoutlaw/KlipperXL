// Neopixel on SPI-shared pin for Prusa XL
//
// Temporarily switches a pin from SPI AF mode to GPIO output for
// WS2812 bit-bang, then restores AF mode.  This allows the ILI9488
// display (SPI6 MOSI on PG14) and WS2812 LED strip (via PE9 mux on
// PG14) to coexist on the same physical pin.
//
// The hardware mux (PE9) is controlled atomically within the MCU
// command to prevent race conditions with display SPI traffic.
//
// Copyright (C) 2026  Richard Crook
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>
#include "autoconf.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "generic/armcm_boot.h"
#include "stm32/internal.h"


/****************************************************************
 * WS2812 timing (matches neopixel.c)
 ****************************************************************/

static uint32_t
nsecs_to_ticks_nps(uint32_t ns)
{
    return DIV_ROUND_UP(timer_from_us(ns * 1000), 1000000);
}

// Minimum HIGH time for a "1" bit to be reliably detected
#define NPS_PULSE_LONG  nsecs_to_ticks_nps(800)
// Minimum time for any level change to be reliably detected
#define NPS_EDGE_MIN    nsecs_to_ticks_nps(200)
// Minimum average total bit time (HIGH + LOW)
#define NPS_BIT_MIN     nsecs_to_ticks_nps(1250)


/****************************************************************
 * Neopixel-over-SPI-pin interface
 ****************************************************************/

struct neopixel_spi_s {
    GPIO_TypeDef *port;
    uint32_t pin_bit;        // 1 << pin_number
    uint32_t pin_shift;      // pin_number * 2 (for MODER)
    struct gpio_out mux_pin; // PE9 hardware mux control
    uint32_t mux_pin_num;    // raw pin number for deferred setup
    uint8_t mux_ready;       // 1 after first gpio_out_setup
    uint32_t last_req_time;
    uint32_t bit_max_ticks;
    uint32_t reset_min_ticks;
    uint16_t data_size;
    uint8_t data[0];
};

void
command_config_neopixel_spi(uint32_t *args)
{
    uint32_t pin = args[1];
    uint32_t mux = args[2];
    uint16_t data_size = args[3];
    if (data_size & 0x8000)
        shutdown("Invalid neopixel_spi data_size");
    struct neopixel_spi_s *n = oid_alloc(
        args[0], command_config_neopixel_spi,
        sizeof(*n) + data_size);
    n->port = gpio_pin_to_regs(pin);
    n->pin_bit = GPIO2BIT(pin);
    n->pin_shift = (pin % 16) * 2;
    // Defer PE9 gpio_out_setup to first send — configuring PE9 during
    // MCU config phase (before display init) breaks the ILI9488.
    // Mux truth table (from Prusa led_lcd_cs_selector.cpp):
    //   CS=LOW,  PE9=LOW  -> LCD only (safe for display SPI)
    //   CS=LOW,  PE9=HIGH -> LCD + side LEDs (corrupts init!)
    //   CS=HIGH, PE9=LOW  -> front LED strip
    //   CS=HIGH, PE9=HIGH -> side LED strip
    n->mux_pin_num = mux;
    n->mux_ready = 0;
    n->data_size = data_size;
    n->bit_max_ticks = args[4];
    n->reset_min_ticks = args[5];
    n->last_req_time = timer_read_time();
}
DECL_COMMAND(command_config_neopixel_spi,
             "config_neopixel_spi oid=%c gpio=%u mux_gpio=%u data_size=%hu"
             " bit_max_ticks=%u reset_min_ticks=%u");

void
command_neopixel_spi_update(uint32_t *args)
{
    uint8_t oid = args[0];
    struct neopixel_spi_s *n = oid_lookup(oid, command_config_neopixel_spi);
    uint_fast16_t pos = args[1];
    uint_fast8_t data_len = args[2];
    uint8_t *data = command_decode_ptr(args[3]);
    if (pos & 0x8000 || pos + data_len > n->data_size)
        shutdown("Invalid neopixel_spi update");
    memcpy(&n->data[pos], data, data_len);
}
DECL_COMMAND(command_neopixel_spi_update,
             "neopixel_spi_update oid=%c pos=%hu data=%*s");

static int
neopixel_spi_send_data(struct neopixel_spi_s *n)
{
    GPIO_TypeDef *port = n->port;
    uint32_t bit = n->pin_bit;
    uint32_t shift = n->pin_shift;
    uint32_t moder_mask = ~(3u << shift);

    // Lazy init of PE9 mux pin — deferred from config to avoid
    // interfering with display init that runs between config and first send.
    if (!n->mux_ready) {
        n->mux_pin = gpio_out_setup(n->mux_pin_num, 0);
        n->mux_ready = 1;
    }

    // Wait for reset time since last send
    uint32_t last = n->last_req_time, rmt = n->reset_min_ticks;
    uint32_t cur = timer_read_time();
    while (cur - last < rmt) {
        irq_poll();
        cur = timer_read_time();
    }

    // Switch pin from SPI AF (MODER=10) to GPIO output (MODER=01).
    // Set output LOW first to avoid glitch on the data line.
    port->BSRR = bit << 16;
    irqstatus_t flag = irq_save();
    port->MODER = (port->MODER & moder_mask) | (1u << shift);
    irq_restore(flag);

    // Select side LEDs: PE9=HIGH (must be after GPIO switch to avoid
    // SPI peripheral glitch reaching WS2812 during MODER transition).
    // CS=HIGH (SPI idle) + PE9=HIGH -> side LED strip per Prusa mux.
    gpio_out_write(n->mux_pin, 1);

    // Bit-bang WS2812 data (logic from neopixel.c send_data)
    struct gpio_out g = { .regs = port, .bit = bit };
    uint8_t *data = n->data;
    uint_fast16_t data_len = n->data_size;
    uint32_t last_start = timer_read_time();
    uint32_t bit_max_ticks = n->bit_max_ticks;

    while (data_len--) {
        uint_fast8_t byte = *data++;
        uint_fast8_t bits = 8;
        while (bits--) {
            if (byte & 0x80) {
                // Long pulse ("1" bit)
                while (timer_read_time() - last_start < NPS_BIT_MIN)
                    ;
                irq_disable();
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(g);
                irq_enable();

                if (start - last_start > bit_max_ticks)
                    goto fail;
                last_start = start;
                byte <<= 1;

                while (timer_read_time() - start < NPS_PULSE_LONG)
                    ;
                irq_disable();
                gpio_out_toggle_noirq(g);
                irq_enable();

                {
                    uint32_t t = timer_read_time();
                    while (timer_read_time() - t < NPS_EDGE_MIN)
                        ;
                }
            } else {
                // Short pulse ("0" bit)
                while (timer_read_time() - last_start < NPS_BIT_MIN)
                    ;
                irq_disable();
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(g);
                while (timer_read_time() - start < NPS_EDGE_MIN)
                    ;
                gpio_out_toggle_noirq(g);
                irq_enable();

                if (start - last_start > bit_max_ticks)
                    goto fail;
                last_start = start;
                byte <<= 1;
            }
        }
    }

    // Hold data line LOW for WS2812 latch time (>50us) so all chips
    // in the chain latch their data before we restore SPI AF mode.
    {
        uint32_t latch_start = timer_read_time();
        while (timer_read_time() - latch_start < rmt)
            irq_poll();
    }

    n->last_req_time = timer_read_time();

    // Deselect side LEDs (PE9=LOW) BEFORE restoring SPI AF mode to
    // prevent SPI peripheral glitch from reaching WS2812.
    gpio_out_write(n->mux_pin, 0);

    // Restore SPI AF mode (MODER=10)
    flag = irq_save();
    port->MODER = (port->MODER & moder_mask) | (2u << shift);
    irq_restore(flag);
    return 0;

fail:
    // Ensure pin LOW, wait for latch, deselect LEDs, restore AF mode
    gpio_out_write(g, 0);
    {
        uint32_t latch_start = timer_read_time();
        while (timer_read_time() - latch_start < rmt)
            irq_poll();
    }
    n->last_req_time = timer_read_time();
    gpio_out_write(n->mux_pin, 0);
    flag = irq_save();
    port->MODER = (port->MODER & moder_mask) | (2u << shift);
    irq_restore(flag);
    return -1;
}

void
command_neopixel_spi_send(uint32_t *args)
{
    uint8_t oid = args[0];
    struct neopixel_spi_s *n = oid_lookup(oid, command_config_neopixel_spi);
    int ret = neopixel_spi_send_data(n);
    sendf("neopixel_spi_result oid=%c success=%c", oid, ret ? 0 : 1);
}
DECL_COMMAND(command_neopixel_spi_send, "neopixel_spi_send oid=%c");


/****************************************************************
 * Stroboscope — TIM13 ISR driven autonomous LED strobe
 *
 * Bit-bangs 6 bytes of WS2812 data (2 drivers) at the target
 * frequency. Driver 1 green channel = white LED.
 * ~108µs per send, 1% CPU at 100Hz. Fine for tuning, not printing.
 ****************************************************************/

static struct {
    struct neopixel_spi_s *nps;
    uint8_t active;
    uint8_t phase;          // 0=off, 1=on
    uint32_t on_arr;        // TIM13 reload for ON duration
    uint32_t off_arr;       // TIM13 reload for OFF duration
    uint8_t data_on[6];     // WS2812: driver 0 off, driver 1 white ON
    uint8_t data_off[6];    // WS2812: driver 0 off, driver 1 white OFF
} strobe;

// Stripped bit-bang for ISR context — no irq_poll, no irq_save.
// Caller must ensure interrupts are disabled or priority is set.
static void
strobe_bitbang(struct neopixel_spi_s *n, uint8_t *data, uint16_t len)
{
    GPIO_TypeDef *port = n->port;
    uint32_t bit = n->pin_bit;
    uint32_t shift = n->pin_shift;
    uint32_t moder_mask = ~(3u << shift);

    // Switch PG14: SPI AF -> GPIO output, pin LOW
    port->BSRR = bit << 16;
    port->MODER = (port->MODER & moder_mask) | (1u << shift);

    // PE9=HIGH (select side strip)
    gpio_out_write(n->mux_pin, 1);

    // Bit-bang WS2812 (same timing as neopixel_spi_send_data)
    struct gpio_out g = { .regs = port, .bit = bit };
    uint32_t last_start = timer_read_time();

    while (len--) {
        uint_fast8_t byte = *data++;
        uint_fast8_t bits = 8;
        while (bits--) {
            if (byte & 0x80) {
                // "1" bit: long HIGH pulse
                while (timer_read_time() - last_start < NPS_BIT_MIN)
                    ;
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(g);
                last_start = start;
                byte <<= 1;
                while (timer_read_time() - start < NPS_PULSE_LONG)
                    ;
                gpio_out_toggle_noirq(g);
                uint32_t t = timer_read_time();
                while (timer_read_time() - t < NPS_EDGE_MIN)
                    ;
            } else {
                // "0" bit: short HIGH pulse
                while (timer_read_time() - last_start < NPS_BIT_MIN)
                    ;
                uint32_t start = timer_read_time();
                gpio_out_toggle_noirq(g);
                while (timer_read_time() - start < NPS_EDGE_MIN)
                    ;
                gpio_out_toggle_noirq(g);
                last_start = start;
                byte <<= 1;
            }
        }
    }

    // Latch (>50µs)
    uint32_t latch = timer_read_time();
    while (timer_read_time() - latch < n->reset_min_ticks)
        ;

    // PE9=LOW, restore SPI AF (MODER=10)
    gpio_out_write(n->mux_pin, 0);
    port->MODER = (port->MODER & moder_mask) | (2u << shift);
}

void
TIM8_UP_TIM13_IRQHandler(void)
{
    // TIM13 shares IRQ with TIM8_UP — check TIM13 first
    if (TIM13->SR & TIM_SR_UIF) {
        TIM13->SR = ~TIM_SR_UIF;

        if (!strobe.active || !strobe.nps || !strobe.nps->mux_ready)
            return;

        if (!strobe.phase) {
            // OFF -> ON
            strobe_bitbang(strobe.nps, strobe.data_on, 6);
            strobe.phase = 1;
            TIM13->ARR = strobe.on_arr;
        } else {
            // ON -> OFF
            strobe_bitbang(strobe.nps, strobe.data_off, 6);
            strobe.phase = 0;
            TIM13->ARR = strobe.off_arr;
        }
    }

    // Chain to TIM8_UP handler if burst stepping is active
    if (TIM8->SR & TIM_SR_UIF) {
        TIM8->SR = ~TIM_SR_UIF;
        // TIM8_UP is handled by DMA, not by ISR — just clear the flag
    }
}

void
command_neopixel_spi_strobe(uint32_t *args)
{
    uint8_t oid = args[0];
    uint16_t freq = args[1];   // Hz, 0 = stop
    uint8_t duty = args[2];    // 0-255, duty cycle (35 = Prusa 13.7%)

    if (freq == 0) {
        // Stop strobe
        TIM13->CR1 &= ~TIM_CR1_CEN;
        TIM13->DIER &= ~TIM_DIER_UIE;
        NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
        strobe.active = 0;

        // Send one final OFF frame
        if (strobe.nps && strobe.nps->mux_ready) {
            irqstatus_t flag = irq_save();
            strobe_bitbang(strobe.nps, strobe.data_off, 6);
            irq_restore(flag);

            // Leave PG14 in GPIO output mode (MODER=01) and mux HIGH so the
            // standard Klipper neopixel driver can drive the LED strip after
            // strobe stops. strobe_bitbang leaves the pin in SPI AF mode with
            // mux LOW, which would block standard neopixel BSRR writes from
            // reaching the WS2812 data line.
            struct neopixel_spi_s *n = strobe.nps;
            GPIO_TypeDef *port = n->port;
            uint32_t shift = n->pin_shift;
            uint32_t moder_mask = ~(3u << shift);
            flag = irq_save();
            port->MODER = (port->MODER & moder_mask) | (1u << shift);
            irq_restore(flag);
            gpio_out_write(n->mux_pin, 1);
        }
        return;
    }

    struct neopixel_spi_s *n = oid_lookup(oid, command_config_neopixel_spi);
    strobe.nps = n;

    // WS2812 data: 6 bytes = 2 drivers × 3 (GRB wire order)
    // Driver 0 (color strip): all off
    // Driver 1: G=white, R=0(fan off), B=0
    memset(strobe.data_on, 0, 6);
    memset(strobe.data_off, 0, 6);
    strobe.data_on[3] = 255;   // Driver 1 G = white LED full brightness

    // TIM13: APB1 timer, 84MHz clock
    // PSC=839 -> 100kHz (10µs per tick)
    RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
    TIM13->CR1 = 0;
    TIM13->PSC = 839;
    TIM13->DIER = 0;

    // Compute ON/OFF durations
    uint32_t total_ticks = 100000 / freq;   // 100kHz / freq
    uint32_t on_ticks = (total_ticks * duty) / 255;
    if (on_ticks < 15) on_ticks = 15;       // min 150µs for bit-bang
    uint32_t off_ticks = total_ticks - on_ticks;
    if (off_ticks < 15) off_ticks = 15;

    strobe.on_arr = on_ticks;
    strobe.off_arr = off_ticks;
    strobe.phase = 0;
    strobe.active = 1;

    // Ensure mux pin is ready
    if (!n->mux_ready) {
        n->mux_pin = gpio_out_setup(n->mux_pin_num, 0);
        n->mux_ready = 1;
    }

    // Start TIM13
    TIM13->ARR = off_ticks;     // first phase is OFF->ON
    TIM13->CNT = 0;
    TIM13->SR = 0;
    TIM13->DIER = TIM_DIER_UIE;
    NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 2);
    NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    TIM13->CR1 = TIM_CR1_CEN;
}
DECL_COMMAND(command_neopixel_spi_strobe,
             "neopixel_spi_strobe oid=%c freq=%hu duty=%c");
DECL_ARMCM_IRQ(TIM8_UP_TIM13_IRQHandler, TIM8_UP_TIM13_IRQn);
