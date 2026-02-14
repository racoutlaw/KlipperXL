// XlKlipper - MODBUS RTU Master for Prusa XL
// Copyright (C) 2026 Richard Crook
//
// Based on Klipper 3D Printer Firmware
//   Copyright (C) 2016-2024 Kevin O'Connor <kevin@koconnor.net>
// Portions derived from Prusa-Firmware-Buddy
//   Copyright (C) 2019-2024 Prusa Research a.s. - www.prusa3d.com
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.
//
// MODBUS RTU master using USART3 on STM32F407 (XLBuddy board)
// RS485: TX=PD8, RX=PD9 (AF7), Direction=PG1, 230400 baud 8N1

#include <string.h>
#include "autoconf.h"
#include "board/armcm_boot.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "internal.h"

// Only compile for STM32F4
#if CONFIG_MACH_STM32F4

/****************************************************************
 * Configuration
 ****************************************************************/

#define MODBUS_BAUD             230400
#define MODBUS_TX_PIN           GPIO('D', 8)
#define MODBUS_RX_PIN           GPIO('D', 9)
#define MODBUS_DIR_PIN          GPIO('G', 1)
#define MODBUS_AF               7

// Timing (in timer ticks)
#define MODBUS_CHAR_TIME_US     44      // ~1 char at 230400 baud
#define MODBUS_FRAME_GAP_US     (MODBUS_CHAR_TIME_US * 4)  // 3.5 char times
#define MODBUS_TIMEOUT_MS       50

// Buffer sizes
#define MODBUS_BUF_SIZE         256

/****************************************************************
 * State
 ****************************************************************/

static struct {
    // Hardware
    USART_TypeDef *usart;
    struct gpio_out dir_pin;
    
    // TX state
    uint8_t tx_buf[MODBUS_BUF_SIZE];
    volatile uint16_t tx_pos;
    volatile uint16_t tx_len;
    
    // RX state
    uint8_t rx_buf[MODBUS_BUF_SIZE];
    volatile uint16_t rx_pos;
    volatile uint32_t rx_last_time;
    
    // Transaction state
    volatile uint8_t state;  // 0=idle, 1=tx, 2=rx, 3=done
    volatile uint8_t result; // 0=ok, 1=timeout, 2=crc, 3=exception
    
    // Statistics
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t crc_errors;
    
    // Init flag
    uint8_t initialized;
} mb;

/****************************************************************
 * CRC16 (MODBUS polynomial 0xA001, reflected)
 ****************************************************************/

static uint16_t
crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= byte;
    for (int i = 0; i < 8; i++) {
        if (crc & 1)
            crc = (crc >> 1) ^ 0xA001;
        else
            crc >>= 1;
    }
    return crc;
}

static uint16_t
crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    while (len--)
        crc = crc16_update(crc, *data++);
    return crc;
}

/****************************************************************
 * RS485 Direction Control
 ****************************************************************/

static inline void
rs485_tx_mode(void)
{
    gpio_out_write(mb.dir_pin, 1);
}

static inline void
rs485_rx_mode(void)
{
    gpio_out_write(mb.dir_pin, 0);
}

/****************************************************************
 * USART3 Interrupt Handler
 ****************************************************************/

void
USART3_IRQHandler(void)
{
    uint32_t sr = mb.usart->SR;
    
    // TX Empty - send next byte
    if ((sr & USART_SR_TXE) && (mb.usart->CR1 & USART_CR1_TXEIE)) {
        if (mb.tx_pos < mb.tx_len) {
            mb.usart->DR = mb.tx_buf[mb.tx_pos++];
        } else {
            // Done sending, wait for TX complete
            mb.usart->CR1 &= ~USART_CR1_TXEIE;
            mb.usart->CR1 |= USART_CR1_TCIE;
        }
    }
    
    // TX Complete - switch to RX mode
    if ((sr & USART_SR_TC) && (mb.usart->CR1 & USART_CR1_TCIE)) {
        mb.usart->CR1 &= ~USART_CR1_TCIE;
        rs485_rx_mode();
        mb.state = 2;  // Now receiving
        mb.rx_pos = 0;
        mb.rx_last_time = timer_read_time();
    }
    
    // RX Not Empty - receive byte
    if (sr & USART_SR_RXNE) {
        uint8_t data = mb.usart->DR;
        if (mb.state == 2 && mb.rx_pos < MODBUS_BUF_SIZE) {
            mb.rx_buf[mb.rx_pos++] = data;
            mb.rx_last_time = timer_read_time();
        }
    }
    
    // Handle errors (clear by reading SR then DR)
    if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
        (void)mb.usart->DR;
    }
}

/****************************************************************
 * Initialization
 ****************************************************************/

static void
modbus_init(void)
{
    if (mb.initialized)
        return;
    
    mb.usart = USART3;
    
    // Enable clocks
    enable_pclock((uint32_t)USART3);
    
    // Configure GPIO
    gpio_peripheral(MODBUS_TX_PIN, GPIO_FUNCTION(MODBUS_AF) | GPIO_HIGH_SPEED, 0);
    gpio_peripheral(MODBUS_RX_PIN, GPIO_FUNCTION(MODBUS_AF), 1);
    mb.dir_pin = gpio_out_setup(MODBUS_DIR_PIN, 0);  // Start in RX mode
    
    // Configure USART
    uint32_t pclk = get_pclock_frequency((uint32_t)USART3);
    uint32_t div = (pclk + MODBUS_BAUD / 2) / MODBUS_BAUD;
    mb.usart->BRR = ((div / 16) << 4) | (div % 16);
    mb.usart->CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE;
    mb.usart->CR2 = 0;
    mb.usart->CR3 = 0;
    
    // Enable interrupt
    armcm_enable_irq(USART3_IRQHandler, USART3_IRQn, 1);
    
    mb.initialized = 1;
}

/****************************************************************
 * MODBUS Transaction
 ****************************************************************/

// Send a MODBUS request and wait for response
// Returns: 0=ok, 1=timeout, 2=crc error, 3=exception
static uint8_t
modbus_transact(uint8_t addr, uint8_t *pdu, uint8_t pdu_len,
                uint8_t *resp, uint8_t *resp_len, uint8_t max_resp)
{
    if (!mb.initialized)
        modbus_init();
    
    if (mb.state != 0)
        return 1;  // Busy
    
    // Build frame: [addr][pdu][crc16]
    mb.tx_buf[0] = addr;
    memcpy(&mb.tx_buf[1], pdu, pdu_len);
    uint16_t frame_len = 1 + pdu_len;
    uint16_t crc = crc16(mb.tx_buf, frame_len);
    mb.tx_buf[frame_len++] = crc & 0xFF;
    mb.tx_buf[frame_len++] = (crc >> 8) & 0xFF;
    
    // Start transmission
    mb.tx_len = frame_len;
    mb.tx_pos = 0;
    mb.rx_pos = 0;
    mb.state = 1;
    mb.result = 0;
    mb.tx_count++;
    
    irq_disable();
    rs485_tx_mode();
    mb.usart->DR = mb.tx_buf[mb.tx_pos++];
    mb.usart->CR1 |= USART_CR1_TXEIE;
    irq_enable();
    
    // Wait for response with timeout
    uint32_t timeout = timer_read_time() + timer_from_us(MODBUS_TIMEOUT_MS * 1000);
    
    while (mb.state != 0) {
        irq_poll();
        
        // Check timeout
        if (timer_is_before(timeout, timer_read_time())) {
            mb.state = 0;
            mb.timeout_count++;
            return 1;  // Timeout
        }
        
        // Check for end of response (silence detection)
        if (mb.state == 2 && mb.rx_pos > 0) {
            uint32_t gap = timer_read_time() - mb.rx_last_time;
            if (gap > timer_from_us(MODBUS_FRAME_GAP_US)) {
                mb.state = 3;  // Frame complete
                break;
            }
        }
    }
    
    // Validate response
    if (mb.rx_pos < 4) {
        mb.state = 0;
        return 1;  // Too short
    }
    
    // Check CRC
    uint16_t rx_crc = crc16(mb.rx_buf, mb.rx_pos - 2);
    uint16_t recv_crc = mb.rx_buf[mb.rx_pos - 2] | (mb.rx_buf[mb.rx_pos - 1] << 8);
    if (rx_crc != recv_crc) {
        mb.state = 0;
        mb.crc_errors++;
        return 2;  // CRC error
    }
    
    // Check for exception
    if (mb.rx_buf[1] & 0x80) {
        mb.state = 0;
        return 3;  // Exception
    }
    
    // Copy response (PDU only)
    uint8_t rlen = mb.rx_pos - 3;  // -1 addr, -2 crc
    if (rlen > max_resp)
        rlen = max_resp;
    memcpy(resp, &mb.rx_buf[1], rlen);
    *resp_len = rlen;
    
    mb.rx_count++;
    mb.state = 0;
    return 0;
}

/****************************************************************
 * Klipper Commands
 ****************************************************************/

DECL_CONSTANT("HAVE_MODBUS", 1);

// Initialize MODBUS
void
command_config_modbus(uint32_t *args)
{
    modbus_init();
}
DECL_COMMAND(command_config_modbus, "config_modbus");

// Read input registers (function code 0x04)
void
command_modbus_read(uint32_t *args)
{
    uint8_t addr = args[0];
    uint16_t start = args[1];
    uint8_t count = args[2];
    
    if (count > 32) count = 32;
    
    uint8_t pdu[5];
    pdu[0] = 0x04;  // Read Input Registers
    pdu[1] = (start >> 8) & 0xFF;
    pdu[2] = start & 0xFF;
    pdu[3] = 0;
    pdu[4] = count;
    
    uint8_t resp[128];
    uint8_t resp_len = 0;
    
    uint8_t status = modbus_transact(addr, pdu, 5, resp, &resp_len, sizeof(resp));
    
    if (status == 0 && resp_len >= 1 + count * 2) {
        // Send register data back (skip byte count)
        sendf("modbus_result addr=%c status=%c data=%*s",
              addr, status, count * 2, &resp[1]);
    } else {
        sendf("modbus_result addr=%c status=%c data=%*s",
              addr, status, 0, "");
    }
}
DECL_COMMAND(command_modbus_read, "modbus_read addr=%c start=%hu count=%c");

// Write single register (function code 0x06)
void
command_modbus_write(uint32_t *args)
{
    uint8_t addr = args[0];
    uint16_t reg = args[1];
    uint16_t value = args[2];
    
    uint8_t pdu[5];
    pdu[0] = 0x06;  // Write Single Register
    pdu[1] = (reg >> 8) & 0xFF;
    pdu[2] = reg & 0xFF;
    pdu[3] = (value >> 8) & 0xFF;
    pdu[4] = value & 0xFF;
    
    uint8_t resp[8];
    uint8_t resp_len = 0;
    
    uint8_t status = modbus_transact(addr, pdu, 5, resp, &resp_len, sizeof(resp));
    
    sendf("modbus_write_result addr=%c reg=%hu status=%c", addr, reg, status);
}
DECL_COMMAND(command_modbus_write, "modbus_write addr=%c reg=%hu value=%hu");

// Write coil (function code 0x05)
void
command_modbus_coil(uint32_t *args)
{
    uint8_t addr = args[0];
    uint16_t coil = args[1];
    uint8_t value = args[2];
    
    uint8_t pdu[5];
    pdu[0] = 0x05;  // Write Single Coil
    pdu[1] = (coil >> 8) & 0xFF;
    pdu[2] = coil & 0xFF;
    pdu[3] = value ? 0xFF : 0x00;
    pdu[4] = 0x00;
    
    uint8_t resp[8];
    uint8_t resp_len = 0;
    
    uint8_t status = modbus_transact(addr, pdu, 5, resp, &resp_len, sizeof(resp));
    
    sendf("modbus_coil_result addr=%c coil=%hu status=%c", addr, coil, status);
}
DECL_COMMAND(command_modbus_coil, "modbus_coil addr=%c coil=%hu value=%c");

// Get MODBUS statistics
void
command_modbus_stats(uint32_t *args)
{
    sendf("modbus_stats tx=%u rx=%u timeout=%u crc=%u",
          mb.tx_count, mb.rx_count, mb.timeout_count, mb.crc_errors);
}
DECL_COMMAND(command_modbus_stats, "modbus_stats");

#endif // CONFIG_MACH_STM32F4
