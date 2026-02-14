// KlipperXL - MODBUS Master + Puppy Bootloader for Prusa XL
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
// For STM32F407 (XLBuddy board)

#include <string.h>
#include "autoconf.h"
#include "board/gpio.h"
#include "board/irq.h"
#include "board/misc.h"
#include "basecmd.h"
#include "command.h"
#include "sched.h"
#include "trsync.h"
#include "board/internal.h"
#include "board/armcm_boot.h"

// Forward declarations
void USART3_IRQHandler(void);
static void puppy_boot_sequence(void);

/****************************************************************
 * Configuration
 ****************************************************************/

// RS485 UART for Puppy communication
#define MODBUS_USART           USART3
#define MODBUS_USART_IRQn      USART3_IRQn
#define MODBUS_BAUD            230400

// RS485 direction control (active high = TX)
#define RS485_DIR_PORT         GPIOG
#define RS485_DIR_PIN          1

// I2C2 for PCA9557 (PF0=SDA, PF1=SCL)
#define PCA9557_I2C            I2C2
#define PCA9557_ADDR           0x19  // 7-bit address (0x32 >> 1)

// PCA9557 registers
#define PCA9557_REG_INPUT      0x00
#define PCA9557_REG_OUTPUT     0x01
#define PCA9557_REG_POLARITY   0x02
#define PCA9557_REG_CONFIG     0x03

// Side filament sensors (sandwich board, ADC3 via mux)
// Matches Prusa XLBuddy hardware: 2x2 analog mux on ADC3
// ADC channels: PC0 = ADC3_IN10 (mux_x), PF6 = ADC3_IN4 (mux_y)
// Mux select: PF12 = setA, PG6 = setB
// Mux positions:
//   pos 0 (A=0,B=0): X=sfs1(T0), Y=sfs2(T1)
//   pos 1 (A=1,B=0): X=sfs3(T2), Y=sfs4(unused)
//   pos 2 (A=0,B=1): X=sfs5(T4), Y=sfs6(T3)
//   pos 3 (A=1,B=1): X=sandwich_temp, Y=ambient_temp
#define SFS_MUX_A_PORT    GPIOF
#define SFS_MUX_A_PIN     12
#define SFS_MUX_B_PORT    GPIOG
#define SFS_MUX_B_PIN     6
#define SFS_ADC_X_CHANNEL 10    // PC0 = ADC3 channel 10
#define SFS_ADC_Y_CHANNEL 4     // PF6 = ADC3 channel 4

// Bootloader protocol
#define BOOTLOADER_CMD_GET_PROTOCOL_VERSION  0x00
#define BOOTLOADER_CMD_SET_ADDRESS           0x01
#define BOOTLOADER_CMD_GET_HARDWARE_INFO     0x03
#define BOOTLOADER_CMD_START_APPLICATION     0x05
#define BOOTLOADER_CMD_GET_FINGERPRINT       0x0E
#define BOOTLOADER_CMD_COMPUTE_FINGERPRINT   0x0F

#define BOOTLOADER_ADDR_DEFAULT    0x00
#define BOOTLOADER_ADDR_FIRST      0x0A
#define MODBUS_ADDR_OFFSET         0x1A

// Buffer sizes
#define MODBUS_TX_BUF_SIZE     256
#define MODBUS_RX_BUF_SIZE     256

// Timeouts - must stay under Klipper's ~500ms watchdog
#define MODBUS_RESPONSE_TIMEOUT_MS  100    // 100ms max total response wait
#define MODBUS_INTER_FRAME_US       3000   // 3ms inter-frame gap between transactions
#define MODBUS_BYTE_TIMEOUT_US      100000 // 100ms max gap between bytes within a response
#define MODBUS_FIRST_BYTE_US        100000 // 100ms wait for first response byte
#define BOOTLOADER_TIMEOUT_MS       400    // 400ms for bootloader commands

/****************************************************************
 * Error Codes
 ****************************************************************/

#define MODBUS_OK              0
#define MODBUS_ERR_TIMEOUT     1
#define MODBUS_ERR_CRC         2
#define MODBUS_ERR_EXCEPTION   3
#define MODBUS_ERR_FRAME       4
#define MODBUS_ERR_BUSY        5

/****************************************************************
 * Global State
 ****************************************************************/

static struct {
    uint8_t tx_buf[MODBUS_TX_BUF_SIZE];
    uint8_t rx_buf[MODBUS_RX_BUF_SIZE];
    volatile uint16_t tx_pos;
    volatile uint16_t tx_len;
    volatile uint16_t rx_pos;
    volatile uint8_t state;
    volatile uint8_t error;
    uint32_t last_activity;
    uint32_t rx_start_time;
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
    uint8_t initialized;
    uint8_t dwarfs_booted;
    uint8_t discovered_dwarfs;  // Bitmask of discovered dwarfs
    uint32_t rx_total_bytes;    // Total bytes received by ISR (debug)
    uint32_t rx_errors;         // Total UART errors (debug)
    uint32_t rx_state0_bytes;   // Bytes received in state 0 (idle)
    uint32_t rx_state1_bytes;   // Bytes received in state 1 (TX) - echo
    uint32_t rx_state2_bytes;   // Bytes received in state 2 (RX) - stored
    uint32_t rx_max_pos;        // Max rx_pos ever seen
} modbus;

/****************************************************************
 * CRC16 Table (polynomial 0xA001, init 0x0000 for bootloader)
 ****************************************************************/

static const uint16_t crc_table[256] = {
    0x0000, 0xC0C1, 0xC181, 0x0140, 0xC301, 0x03C0, 0x0280, 0xC241,
    0xC601, 0x06C0, 0x0780, 0xC741, 0x0500, 0xC5C1, 0xC481, 0x0440,
    0xCC01, 0x0CC0, 0x0D80, 0xCD41, 0x0F00, 0xCFC1, 0xCE81, 0x0E40,
    0x0A00, 0xCAC1, 0xCB81, 0x0B40, 0xC901, 0x09C0, 0x0880, 0xC841,
    0xD801, 0x18C0, 0x1980, 0xD941, 0x1B00, 0xDBC1, 0xDA81, 0x1A40,
    0x1E00, 0xDEC1, 0xDF81, 0x1F40, 0xDD01, 0x1DC0, 0x1C80, 0xDC41,
    0x1400, 0xD4C1, 0xD581, 0x1540, 0xD701, 0x17C0, 0x1680, 0xD641,
    0xD201, 0x12C0, 0x1380, 0xD341, 0x1100, 0xD1C1, 0xD081, 0x1040,
    0xF001, 0x30C0, 0x3180, 0xF141, 0x3300, 0xF3C1, 0xF281, 0x3240,
    0x3600, 0xF6C1, 0xF781, 0x3740, 0xF501, 0x35C0, 0x3480, 0xF441,
    0x3C00, 0xFCC1, 0xFD81, 0x3D40, 0xFF01, 0x3FC0, 0x3E80, 0xFE41,
    0xFA01, 0x3AC0, 0x3B80, 0xFB41, 0x3900, 0xF9C1, 0xF881, 0x3840,
    0x2800, 0xE8C1, 0xE981, 0x2940, 0xEB01, 0x2BC0, 0x2A80, 0xEA41,
    0xEE01, 0x2EC0, 0x2F80, 0xEF41, 0x2D00, 0xEDC1, 0xEC81, 0x2C40,
    0xE401, 0x24C0, 0x2580, 0xE541, 0x2700, 0xE7C1, 0xE681, 0x2640,
    0x2200, 0xE2C1, 0xE381, 0x2340, 0xE101, 0x21C0, 0x2080, 0xE041,
    0xA001, 0x60C0, 0x6180, 0xA141, 0x6300, 0xA3C1, 0xA281, 0x6240,
    0x6600, 0xA6C1, 0xA781, 0x6740, 0xA501, 0x65C0, 0x6480, 0xA441,
    0x6C00, 0xACC1, 0xAD81, 0x6D40, 0xAF01, 0x6FC0, 0x6E80, 0xAE41,
    0xAA01, 0x6AC0, 0x6B80, 0xAB41, 0x6900, 0xA9C1, 0xA881, 0x6840,
    0x7800, 0xB8C1, 0xB981, 0x7940, 0xBB01, 0x7BC0, 0x7A80, 0xBA41,
    0xBE01, 0x7EC0, 0x7F80, 0xBF41, 0x7D00, 0xBDC1, 0xBC81, 0x7C40,
    0xB401, 0x74C0, 0x7580, 0xB541, 0x7700, 0xB7C1, 0xB681, 0x7640,
    0x7200, 0xB2C1, 0xB381, 0x7340, 0xB101, 0x71C0, 0x7080, 0xB041,
    0x5000, 0x90C1, 0x9181, 0x5140, 0x9301, 0x53C0, 0x5280, 0x9241,
    0x9601, 0x56C0, 0x5780, 0x9741, 0x5500, 0x95C1, 0x9481, 0x5440,
    0x9C01, 0x5CC0, 0x5D80, 0x9D41, 0x5F00, 0x9FC1, 0x9E81, 0x5E40,
    0x5A00, 0x9AC1, 0x9B81, 0x5B40, 0x9901, 0x59C0, 0x5880, 0x9841,
    0x8801, 0x48C0, 0x4980, 0x8941, 0x4B00, 0x8BC1, 0x8A81, 0x4A40,
    0x4E00, 0x8EC1, 0x8F81, 0x4F40, 0x8D01, 0x4DC0, 0x4C80, 0x8C41,
    0x4400, 0x84C1, 0x8581, 0x4540, 0x8701, 0x47C0, 0x4680, 0x8641,
    0x8201, 0x42C0, 0x4380, 0x8341, 0x4100, 0x81C1, 0x8081, 0x4040,
};

/****************************************************************
 * CRC Calculation
 ****************************************************************/

static uint16_t
calc_crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;  // Init 0xFFFF for both bootloader AND MODBUS (CRC16-IBM)
    for (uint16_t i = 0; i < len; i++) {
        crc = (crc >> 8) ^ crc_table[(crc ^ data[i]) & 0xFF];
    }
    return crc;
}

/****************************************************************
 * RS485 Direction Control
 ****************************************************************/

static inline void rs485_tx_enable(void) {
    RS485_DIR_PORT->BSRR = (1 << RS485_DIR_PIN);
}

static inline void rs485_rx_enable(void) {
    RS485_DIR_PORT->BSRR = (1 << (RS485_DIR_PIN + 16));
}

/****************************************************************
 * Simple Delay
 ****************************************************************/

static void delay_ms(uint32_t ms)
{
    uint32_t end = timer_read_time() + timer_from_us(ms * 1000);
    while (timer_is_before(timer_read_time(), end)) {
        // busy wait
    }
}

static void delay_us(uint32_t us)
{
    uint32_t end = timer_read_time() + timer_from_us(us);
    while (timer_is_before(timer_read_time(), end)) {
        // busy wait
    }
}

/****************************************************************
 * Hardware I2C2 for PCA9557 (PF0=SDA, PF1=SCL)
 ****************************************************************/

// I2C pins: PF0 = SDA, PF1 = SCL
#define I2C_SDA_PORT  GPIOF
#define I2C_SDA_PIN   0
#define I2C_SCL_PORT  GPIOF
#define I2C_SCL_PIN   1

static void hw_i2c_init(void)
{
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    
    for (volatile int i = 0; i < 10000; i++) {}  // Wait for clocks
    
    // Configure PF0 (SDA) and PF1 (SCL) for I2C2 (AF4)
    GPIOF->MODER &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOF->MODER |= (2 << (0 * 2)) | (2 << (1 * 2));  // Alternate function
    GPIOF->OTYPER |= (1 << 0) | (1 << 1);  // Open-drain
    GPIOF->PUPDR &= ~((3 << (0 * 2)) | (3 << (1 * 2)));
    GPIOF->PUPDR |= (1 << (0 * 2)) | (1 << (1 * 2));  // Pull-up
    GPIOF->AFR[0] &= ~((0xF << (0 * 4)) | (0xF << (1 * 4)));
    GPIOF->AFR[0] |= (4 << (0 * 4)) | (4 << (1 * 4));  // AF4 = I2C2
    
    // Reset I2C2
    I2C2->CR1 = I2C_CR1_SWRST;
    for (volatile int i = 0; i < 1000; i++) {}
    I2C2->CR1 = 0;
    
    // Configure I2C2: 100kHz from 42MHz APB1
    I2C2->CR2 = 42;  // APB1 = 42MHz
    I2C2->CCR = 210;  // 42MHz / (2 * 100kHz) = 210
    I2C2->TRISE = 43;  // (42MHz / 1MHz) + 1
    
    // Enable I2C
    I2C2->CR1 = I2C_CR1_PE;
}

static int hw_i2c_write(uint8_t addr, uint8_t reg, uint8_t value)
{
    int timeout;
    
    // Wait for bus free
    timeout = 10000;
    while ((I2C2->SR2 & I2C_SR2_BUSY) && --timeout > 0) {}
    if (timeout == 0) return -1;
    
    // Start
    I2C2->CR1 |= I2C_CR1_START;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_SB) && --timeout > 0) {}
    if (timeout == 0) return -2;
    
    // Send address (write)
    I2C2->DR = addr << 1;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_ADDR) && --timeout > 0) {
        if (I2C2->SR1 & I2C_SR1_AF) {
            I2C2->CR1 |= I2C_CR1_STOP;
            I2C2->SR1 &= ~I2C_SR1_AF;
            return -3;  // NACK
        }
    }
    if (timeout == 0) return -4;
    (void)I2C2->SR2;  // Clear ADDR
    
    // Send register
    I2C2->DR = reg;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout > 0) {}
    if (timeout == 0) return -5;
    
    // Send value
    I2C2->DR = value;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_TXE) && --timeout > 0) {}
    if (timeout == 0) return -6;
    timeout = 10000;
    while (!(I2C2->SR1 & I2C_SR1_BTF) && --timeout > 0) {}
    if (timeout == 0) return -7;
    
    // Stop
    I2C2->CR1 |= I2C_CR1_STOP;
    
    return 0;
}

/****************************************************************
 * Bit-bang I2C (kept as backup)
 ****************************************************************/

static void i2c_delay(void) {
    for (volatile int i = 0; i < 50; i++) {}
}

static void i2c_sda_high(void) {
    I2C_SDA_PORT->MODER &= ~(3 << (I2C_SDA_PIN * 2));  // Input (high-Z, pulled up)
}

static void i2c_sda_low(void) {
    I2C_SDA_PORT->BSRR = (1 << (I2C_SDA_PIN + 16));    // Set ODR low first
    I2C_SDA_PORT->MODER &= ~(3 << (I2C_SDA_PIN * 2));
    I2C_SDA_PORT->MODER |= (1 << (I2C_SDA_PIN * 2));   // Then output mode
}

static uint8_t i2c_sda_read(void) {
    return (I2C_SDA_PORT->IDR >> I2C_SDA_PIN) & 1;
}

static void i2c_scl_high(void) {
    // Open drain: set to input mode to release (pull-up pulls high)
    I2C_SCL_PORT->MODER &= ~(3 << (I2C_SCL_PIN * 2));  // Input mode
}

static void i2c_scl_low(void) {
    // Open drain: set to output mode (ODR already 0, so drives low)
    I2C_SCL_PORT->MODER &= ~(3 << (I2C_SCL_PIN * 2));
    I2C_SCL_PORT->MODER |= (1 << (I2C_SCL_PIN * 2));   // Output mode
}

static void i2c_init(void)
{
    // Enable GPIOF clock
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;
    
    // Set ODR low for SDA (so when we switch to output mode, it drives low)
    GPIOF->BSRR = (1 << (I2C_SDA_PIN + 16));  // SDA ODR = 0
    
    // Configure PF0 (SDA) as open-drain output with pull-up
    GPIOF->OTYPER |= (1 << I2C_SDA_PIN);  // Open-drain
    GPIOF->PUPDR &= ~(3 << (I2C_SDA_PIN * 2));
    GPIOF->PUPDR |= (1 << (I2C_SDA_PIN * 2));  // Pull-up
    GPIOF->OSPEEDR |= (3 << (I2C_SDA_PIN * 2));  // High speed
    GPIOF->MODER &= ~(3 << (I2C_SDA_PIN * 2));  // Input mode (high-Z = high)
    
    // Set ODR low for SCL 
    GPIOF->BSRR = (1 << (I2C_SCL_PIN + 16));  // SCL ODR = 0
    
    // Configure PF1 (SCL) as open-drain output with pull-up
    GPIOF->MODER &= ~(3 << (I2C_SCL_PIN * 2));
    GPIOF->MODER |= (1 << (I2C_SCL_PIN * 2));  // Output mode
    GPIOF->OTYPER |= (1 << I2C_SCL_PIN);  // Open-drain
    GPIOF->PUPDR &= ~(3 << (I2C_SCL_PIN * 2));
    GPIOF->PUPDR |= (1 << (I2C_SCL_PIN * 2));  // Pull-up
    GPIOF->OSPEEDR |= (3 << (I2C_SCL_PIN * 2));  // High speed
    
    // Start with both high
    i2c_sda_high();
    i2c_scl_high();
    i2c_delay();
}

static void i2c_start(void)
{
    i2c_sda_high();
    i2c_scl_high();
    i2c_delay();
    i2c_sda_low();
    i2c_delay();
    i2c_scl_low();
    i2c_delay();
}

static void i2c_stop(void)
{
    i2c_sda_low();
    i2c_delay();
    i2c_scl_high();
    i2c_delay();
    i2c_sda_high();
    i2c_delay();
}

static uint8_t i2c_write_byte(uint8_t data)
{
    for (int i = 7; i >= 0; i--) {
        if (data & (1 << i)) {
            i2c_sda_high();
        } else {
            i2c_sda_low();
        }
        i2c_delay();
        i2c_scl_high();
        i2c_delay();
        i2c_scl_low();
        i2c_delay();
    }
    
    // Read ACK
    i2c_sda_high();
    i2c_delay();
    i2c_scl_high();
    i2c_delay();
    uint8_t ack = !i2c_sda_read();  // ACK = SDA low
    i2c_scl_low();
    i2c_delay();
    
    return ack;
}

static void pca9557_write(uint8_t reg, uint8_t value)
{
    i2c_start();
    i2c_write_byte((PCA9557_ADDR << 1) | 0);  // Write
    i2c_write_byte(reg);
    i2c_write_byte(value);
    i2c_stop();
}

static void pca9557_release_all(void)
{
    // Set all outputs LOW (release from reset) - Prusa: low=release, high=hold
    pca9557_write(PCA9557_REG_OUTPUT, 0x00);
    // Configure all pins as outputs
    pca9557_write(PCA9557_REG_CONFIG, 0x00);
}

static void pca9557_reset_all(void)
{
    // Set all outputs HIGH (hold in reset) - Prusa: low=release, high=hold
    pca9557_write(PCA9557_REG_OUTPUT, 0xFF);
}

/****************************************************************
 * USART3 Initialization
 ****************************************************************/

static void modbus_uart_init(void)
{
    if (modbus.initialized)
        return;
    
    // Enable clocks
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

    // Configure PD8 (TX) and PD9 (RX) as alternate function AF7
    GPIOD->MODER &= ~(GPIO_MODER_MODER8 | GPIO_MODER_MODER9);
    GPIOD->MODER |= (2 << GPIO_MODER_MODER8_Pos) | (2 << GPIO_MODER_MODER9_Pos);
    GPIOD->AFR[1] &= ~((0xF << 0) | (0xF << 4));
    GPIOD->AFR[1] |= (7 << 0) | (7 << 4);
    GPIOD->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR8 | GPIO_OSPEEDER_OSPEEDR9;
    // Add pull-up on RX pin
    GPIOD->PUPDR &= ~GPIO_PUPDR_PUPD9;
    GPIOD->PUPDR |= (1 << GPIO_PUPDR_PUPD9_Pos);  // Pull-up

    // Configure PG1 as output for RS485 direction
    GPIOG->MODER &= ~GPIO_MODER_MODER1;
    GPIOG->MODER |= (1 << GPIO_MODER_MODER1_Pos);
    GPIOG->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR1;
    rs485_rx_enable();

    // Configure USART3: 230400 baud @ 42MHz APB1
    MODBUS_USART->CR1 = 0;
    MODBUS_USART->BRR = 182;  // 42000000 / 230400 ≈ 182
    MODBUS_USART->CR1 = USART_CR1_TE | USART_CR1_RE;  // No RXNEIE - use polling
    MODBUS_USART->CR2 = 0;
    MODBUS_USART->CR3 = 0;
    MODBUS_USART->CR1 |= USART_CR1_UE;

    armcm_enable_irq(USART3_IRQHandler, MODBUS_USART_IRQn, 2);

    modbus.initialized = 1;
}

/****************************************************************
 * USART3 Interrupt Handler
 ****************************************************************/

void USART3_IRQHandler(void)
{
    uint32_t sr = MODBUS_USART->SR;

    if ((sr & USART_SR_TXE) && (MODBUS_USART->CR1 & USART_CR1_TXEIE)) {
        if (modbus.tx_pos < modbus.tx_len) {
            MODBUS_USART->DR = modbus.tx_buf[modbus.tx_pos++];
        } else {
            MODBUS_USART->CR1 &= ~USART_CR1_TXEIE;
            MODBUS_USART->CR1 |= USART_CR1_TCIE;
        }
    }

    if ((sr & USART_SR_TC) && (MODBUS_USART->CR1 & USART_CR1_TCIE)) {
        MODBUS_USART->CR1 &= ~USART_CR1_TCIE;
        // Switch to RX mode
        rs485_rx_enable();
        // Small delay for transceiver to settle
        for (volatile int i = 0; i < 100; i++) {}
        // Clear any pending RX data/flags from TX echo
        (void)MODBUS_USART->SR;
        (void)MODBUS_USART->DR;
        // Now ready to receive - polling will handle RX (no RXNEIE)
        modbus.state = 2;  // Receiving
        modbus.rx_pos = 0;
        modbus.rx_start_time = timer_read_time();
        modbus.last_activity = timer_read_time();
    }

    // RX is handled by polling in send_frame_wait - don't touch DR here
}

/****************************************************************
 * Send Frame and Wait for Response
 ****************************************************************/

static int
send_frame_wait(uint8_t *data, uint16_t len, uint8_t *response, uint16_t *resp_len, uint16_t max_resp, uint32_t timeout_ms)
{
    if (!modbus.initialized)
        return MODBUS_ERR_BUSY;
    if (modbus.state != 0)
        return MODBUS_ERR_BUSY;
    if (len > MODBUS_TX_BUF_SIZE)
        return MODBUS_ERR_FRAME;

    // Wait for inter-frame gap
    while (timer_read_time() - modbus.last_activity < timer_from_us(MODBUS_INTER_FRAME_US)) {
        // busy wait
    }

    // Flush any stale UART data before transmitting
    while (MODBUS_USART->SR & USART_SR_RXNE) {
        (void)MODBUS_USART->DR;
    }
    // Clear any error flags
    (void)MODBUS_USART->SR;
    (void)MODBUS_USART->DR;

    // Copy data to TX buffer
    memcpy(modbus.tx_buf, data, len);
    modbus.tx_len = len;
    modbus.tx_pos = 0;
    modbus.rx_pos = 0;
    modbus.state = 1;  // Transmitting
    modbus.error = MODBUS_OK;

    // Start transmission
    irq_disable();
    rs485_tx_enable();
    delay_us(10);  // Small delay for transceiver
    MODBUS_USART->DR = modbus.tx_buf[modbus.tx_pos++];
    MODBUS_USART->CR1 |= USART_CR1_TXEIE;
    irq_enable();

    modbus.tx_count++;

    // Wait for response using pure polling (no RX interrupts)
    uint32_t timeout = timer_read_time() + timer_from_us(timeout_ms * 1000);
    // First byte must arrive within timeout, otherwise no response
    uint32_t first_byte_timeout = timer_read_time() + timer_from_us(MODBUS_FIRST_BYTE_US);

    while (modbus.state != 0) {
        // Overall timeout
        if (timer_is_before(timeout, timer_read_time())) {
            irq_disable();  // Prevent race with ISR

            // CRITICAL FIX: Release DE pin to RX mode!
            // Without this, DE stays high and blocks ALL future RX
            rs485_rx_enable();

            // Disable any pending UART interrupts
            MODBUS_USART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);

            // Clear any pending flags
            (void)MODBUS_USART->SR;
            (void)MODBUS_USART->DR;

            modbus.state = 0;
            modbus.timeout_count++;

            irq_enable();

            if (modbus.rx_pos > 0 && response && resp_len) {
                uint16_t copy_len = modbus.rx_pos;
                if (copy_len > max_resp) copy_len = max_resp;
                memcpy(response, modbus.rx_buf, copy_len);
                *resp_len = copy_len;
            } else if (resp_len) {
                *resp_len = 0;
            }
            return MODBUS_ERR_TIMEOUT;
        }

        // First byte timeout: if nothing received in 200ms, give up
        if (modbus.state == 2 && modbus.rx_pos == 0) {
            if (timer_is_before(first_byte_timeout, timer_read_time())) {
                irq_disable();

                // CRITICAL FIX: Ensure DE pin is in RX mode
                rs485_rx_enable();
                MODBUS_USART->CR1 &= ~(USART_CR1_TXEIE | USART_CR1_TCIE);
                (void)MODBUS_USART->SR;
                (void)MODBUS_USART->DR;

                modbus.state = 0;
                modbus.timeout_count++;

                irq_enable();

                if (resp_len) *resp_len = 0;
                return MODBUS_ERR_TIMEOUT;
            }
        }

        // Poll UART for received bytes
        if (modbus.state == 2) {
            uint32_t sr = MODBUS_USART->SR;
            if (sr & USART_SR_RXNE) {
                uint8_t data = MODBUS_USART->DR;
                modbus.rx_total_bytes++;
                if (modbus.rx_pos < MODBUS_RX_BUF_SIZE) {
                    modbus.rx_buf[modbus.rx_pos++] = data;
                    modbus.last_activity = timer_read_time();
                    if (modbus.rx_pos > modbus.rx_max_pos)
                        modbus.rx_max_pos = modbus.rx_pos;
                }
            } else if (sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) {
                // Clear error by reading SR then DR
                (void)MODBUS_USART->DR;
                modbus.rx_errors++;
            }
        }

        // Protocol-aware response detection:
        // MODBUS exception: [addr][func|0x80][exception_code][crc16] = 5 bytes
        // FC 0x06 (Write Single Register): [addr][func][reg_hi][reg_lo][val_hi][val_lo][crc16] = 8 bytes
        // FC 0x10 (Write Multiple Regs):   [addr][func][reg_hi][reg_lo][qty_hi][qty_lo][crc16] = 8 bytes
        // MODBUS read resp: [addr][func][byte_count][data...][crc16]
        // FC 0x18 (FIFO):   [addr][0x18][byte_count_hi][byte_count_lo][...][crc16]
        // Bootloader resp:  [addr][status][len][data...][crc16]
        if (modbus.state == 2 && modbus.rx_pos >= 3) {
            uint16_t expected_len = 0;
            uint8_t func_code = modbus.rx_buf[1];
            if (func_code & 0x80) {
                // MODBUS exception response - always 5 bytes
                expected_len = 5;
            } else if (func_code == 0x05 || func_code == 0x06 || func_code == 0x0F || func_code == 0x10) {
                // FC 0x05 Write Single Coil / FC 0x06 Write Single Register
                // FC 0x0F Write Multiple Coils / FC 0x10 Write Multiple Registers
                // Response is echo: [addr][func][addr_hi][addr_lo][val_or_qty_hi][val_or_qty_lo][crc16]
                expected_len = 8;
            } else if (func_code == 0x18) {
                // FC 0x18 Read FIFO Queue - 2-byte byte count
                if (modbus.rx_pos >= 4) {
                    uint16_t byte_count = ((uint16_t)modbus.rx_buf[2] << 8)
                                        | modbus.rx_buf[3];
                    expected_len = 4 + byte_count + 2;  // addr+func+bc(2)+data+crc(2)
                }
                // else: need more bytes before we can determine length
            } else {
                // Default: treat byte[2] as payload length
                uint8_t payload_len = modbus.rx_buf[2];
                expected_len = 3 + payload_len + 2;
            }
            if (expected_len > 0) {
                if (expected_len > MODBUS_RX_BUF_SIZE) expected_len = MODBUS_RX_BUF_SIZE;
                if (modbus.rx_pos >= expected_len) {
                    modbus.state = 3;  // Frame complete
                    break;
                }
            }
        }

        // Byte gap timeout: if bytes started but gap > 500ms, frame done
        if (modbus.state == 2 && modbus.rx_pos > 0) {
            if (timer_read_time() - modbus.last_activity > timer_from_us(MODBUS_BYTE_TIMEOUT_US)) {
                modbus.state = 3;  // Frame complete (gap timeout)
                break;
            }
        }
    }

    // Validate CRC on received frame
    if (modbus.rx_pos >= 5) {
        uint16_t expected_crc = calc_crc16(modbus.rx_buf, modbus.rx_pos - 2);
        uint16_t received_crc = modbus.rx_buf[modbus.rx_pos - 2]
                              | (modbus.rx_buf[modbus.rx_pos - 1] << 8);
        if (expected_crc != received_crc) {
            modbus.crc_error_count++;
            modbus.state = 0;
            // Return raw data even on CRC error for debugging
            if (response && resp_len && modbus.rx_pos > 0) {
                uint16_t copy_len = modbus.rx_pos;
                if (copy_len > max_resp) copy_len = max_resp;
                memcpy(response, modbus.rx_buf, copy_len);
                *resp_len = copy_len;
            } else if (resp_len) {
                *resp_len = 0;
            }
            // Inter-frame delay
            uint32_t gap_end = timer_read_time() + timer_from_us(MODBUS_INTER_FRAME_US);
            while (timer_is_before(timer_read_time(), gap_end)) {}
            return MODBUS_ERR_CRC;
        }
    }

    // Copy response
    if (response && resp_len && modbus.rx_pos > 0) {
        uint16_t copy_len = modbus.rx_pos;
        if (copy_len > max_resp) copy_len = max_resp;
        memcpy(response, modbus.rx_buf, copy_len);
        *resp_len = copy_len;
        modbus.rx_count++;
    } else if (resp_len) {
        *resp_len = 0;
    }

    modbus.state = 0;

    // Inter-frame delay before next transaction
    uint32_t gap_end = timer_read_time() + timer_from_us(MODBUS_INTER_FRAME_US);
    while (timer_is_before(timer_read_time(), gap_end)) {}

    return MODBUS_OK;
}

/****************************************************************
 * Bootloader Protocol Functions
 ****************************************************************/

// Build bootloader frame: [addr][cmd][data...][crc16-le]
static uint16_t
build_bootloader_frame(uint8_t *buf, uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t data_len)
{
    buf[0] = addr;
    buf[1] = cmd;
    if (data && data_len > 0) {
        memcpy(&buf[2], data, data_len);
    }
    uint16_t frame_len = 2 + data_len;
    uint16_t crc = calc_crc16(buf, frame_len);
    buf[frame_len] = crc & 0xFF;
    buf[frame_len + 1] = (crc >> 8) & 0xFF;
    return frame_len + 2;
}

// Send bootloader command and get response
static int
bootloader_command(uint8_t addr, uint8_t cmd, uint8_t *data, uint8_t data_len,
                   uint8_t *resp_data, uint8_t *resp_len)
{
    uint8_t frame[64];
    uint16_t frame_len = build_bootloader_frame(frame, addr, cmd, data, data_len);
    
    uint8_t response[64];
    uint16_t response_len = 0;
    
    int ret = send_frame_wait(frame, frame_len, response, &response_len, sizeof(response), BOOTLOADER_TIMEOUT_MS);
    
    if (ret != MODBUS_OK) {
        return ret;
    }
    
    // Parse response: [addr][status][len][data...][crc16]
    if (response_len < 5) {  // Minimum: addr + status + len + crc16
        return MODBUS_ERR_FRAME;
    }
    
    // Verify CRC
    uint16_t expected_crc = calc_crc16(response, response_len - 2);
    uint16_t received_crc = response[response_len - 2] | (response[response_len - 1] << 8);
    if (expected_crc != received_crc) {
        return MODBUS_ERR_CRC;
    }
    
    // Check address matches
    if (response[0] != addr) {
        return MODBUS_ERR_FRAME;
    }
    
    // Return status and data
    uint8_t status = response[1];
    uint8_t data_length = response[2];
    
    if (resp_data && resp_len && data_length > 0) {
        memcpy(resp_data, &response[3], data_length);
        *resp_len = data_length;
    } else if (resp_len) {
        *resp_len = 0;
    }
    
    return status;  // 0 = COMMAND_OK
}

// Discover puppy at address
static int
bootloader_discover(uint8_t addr)
{
    uint8_t resp_data[8];
    uint8_t resp_len = 0;
    
    // Send GET_PROTOCOL_VERSION
    int ret = bootloader_command(addr, BOOTLOADER_CMD_GET_PROTOCOL_VERSION, 
                                  NULL, 0, resp_data, &resp_len);
    
    return (ret == 0) ? 1 : 0;  // 1 = found, 0 = not found
}

// Assign address to puppy
static void
bootloader_assign_address(uint8_t current_addr, uint8_t new_addr)
{
    uint8_t data[1] = { new_addr };
    // This is a no-reply command
    uint8_t frame[8];
    uint16_t frame_len = build_bootloader_frame(frame, current_addr, BOOTLOADER_CMD_SET_ADDRESS, data, 1);
    
    uint8_t response[8];
    uint16_t response_len = 0;
    send_frame_wait(frame, frame_len, response, &response_len, sizeof(response), 10);  // Short timeout, no reply expected
}

// Start application on puppy - PROPER IMPLEMENTATION
// Must first compute fingerprint, get it, then send START_APPLICATION with matching salt+fingerprint
static int
bootloader_start_app(uint8_t addr, uint32_t salt, uint8_t *fingerprint)
{
    // START_APPLICATION needs salt (4 bytes) + fingerprint (32 bytes)
    uint8_t data[36];
    
    // Salt is big-endian
    data[0] = (salt >> 24) & 0xFF;
    data[1] = (salt >> 16) & 0xFF;
    data[2] = (salt >> 8) & 0xFF;
    data[3] = salt & 0xFF;
    
    // Copy fingerprint (32 bytes)
    memcpy(&data[4], fingerprint, 32);
    
    uint8_t resp_data[4];
    uint8_t resp_len = 0;
    
    return bootloader_command(addr, BOOTLOADER_CMD_START_APPLICATION, 
                              data, sizeof(data), resp_data, &resp_len);
}

// Ask puppy to compute fingerprint with given salt
static int
bootloader_compute_fingerprint(uint8_t addr, uint32_t salt)
{
    uint8_t data[4];
    // Salt is big-endian
    data[0] = (salt >> 24) & 0xFF;
    data[1] = (salt >> 16) & 0xFF;
    data[2] = (salt >> 8) & 0xFF;
    data[3] = salt & 0xFF;
    
    // This is a write-only command, but we still wait for ACK
    uint8_t frame[16];
    uint16_t frame_len = build_bootloader_frame(frame, addr, BOOTLOADER_CMD_COMPUTE_FINGERPRINT, data, 4);
    
    uint8_t response[8];
    uint16_t response_len = 0;
    return send_frame_wait(frame, frame_len, response, &response_len, sizeof(response), BOOTLOADER_TIMEOUT_MS);
}

// Get computed fingerprint from puppy
static int
bootloader_get_fingerprint(uint8_t addr, uint8_t *fingerprint, uint8_t offset, uint8_t size)
{
    uint8_t data[2];
    data[0] = offset;
    data[1] = size;
    
    uint8_t resp_len = 0;
    return bootloader_command(addr, BOOTLOADER_CMD_GET_FINGERPRINT,
                              data, 2, fingerprint, &resp_len);
}

/****************************************************************
 * Puppy Boot Sequence (called on MCU startup)
 ****************************************************************/

// Simple pseudo-random generator for salt
static uint32_t simple_rand_state = 0x12345678;
static uint32_t simple_rand(void) {
    simple_rand_state ^= simple_rand_state << 13;
    simple_rand_state ^= simple_rand_state >> 17;
    simple_rand_state ^= simple_rand_state << 5;
    return simple_rand_state;
}

/****************************************************************
 * PCA9557 Reset Pin Helpers
 *
 * Pin mapping: bit0=dwarf6, bit1=dwarf1, bit2=dwarf2, bit3=dwarf3,
 *              bit4=dwarf4, bit5=dwarf5, bit6=fan, bit7=bed
 * HIGH = hold in reset, LOW = release
 ****************************************************************/

#define NUM_DWARFS          5
#define ALL_DWARF_BITS      0x3E  // bits 1-5 for Dwarfs 1-5

// Get the PCA9557 bit for a given Dwarf number (1-5)
static inline uint8_t dwarf_reset_bit(uint8_t dwarf_num) {
    return (1 << dwarf_num);  // Dwarf 1=bit1, 2=bit2, etc.
}

// Set PCA9557 output register (controls reset pins)
static void pca9557_set_output(uint8_t value) {
    hw_i2c_write(PCA9557_ADDR, PCA9557_REG_OUTPUT, value);
    delay_ms(1);
}

static void
puppy_boot_sequence(void)
{
    modbus_uart_init();

    // Seed PRNG with timer value for somewhat random salts
    simple_rand_state ^= timer_read_time();

    modbus.discovered_dwarfs = 0;

    // Step 1: Reset ALL Dwarfs (hold in reset)
    pca9557_set_output(ALL_DWARF_BITS);
    delay_ms(5);

    // Step 2: Release ALL from reset (bootloaders start, all at address 0x00)
    pca9557_set_output(0x00);
    delay_ms(10);  // Give bootloaders time to initialize

    // Step 3: For each Dwarf, assign address, verify fingerprint, start app
    for (uint8_t dwarf = 1; dwarf <= NUM_DWARFS; dwarf++) {
        uint8_t assigned_addr = BOOTLOADER_ADDR_FIRST + (dwarf - 1);  // 0x0A, 0x0B, ...

        // 3a: Assign address - all Dwarfs at 0x00 will accept this
        bootloader_assign_address(BOOTLOADER_ADDR_DEFAULT, assigned_addr);
        delay_ms(50);  // Wait for command to be processed

        // 3b: Reset all OTHER Dwarfs (they go back to bootloader at 0x00 on next release)
        uint8_t others_reset = ALL_DWARF_BITS & ~dwarf_reset_bit(dwarf);
        pca9557_set_output(others_reset);
        delay_ms(5);

        // 3c: Discover this Dwarf at its assigned address
        int found = bootloader_discover(assigned_addr);
        if (!found) {
            // Dwarf not present in this dock, release all and continue
            pca9557_set_output(0x00);
            delay_ms(10);
            continue;
        }

        // 3d: Generate random salt and request fingerprint computation
        uint32_t salt = simple_rand();
        int ret = bootloader_compute_fingerprint(assigned_addr, salt);
        // Accept OK or timeout (Dwarf may not ACK this command)
        if (ret != MODBUS_OK && ret != MODBUS_ERR_TIMEOUT) {
            // COMPUTE_FINGERPRINT failed, release and skip
            pca9557_set_output(0x00);
            delay_ms(10);
            continue;
        }

        // 3e: Wait for SHA256 computation (takes 330-600ms on Dwarf MCU)
        // Poll GET_FINGERPRINT until it succeeds
        uint8_t fingerprint[32];
        int fp_ok = 0;
        for (int attempt = 0; attempt < 20; attempt++) {  // Up to 20 * 100ms = 2000ms
            delay_ms(100);
            ret = bootloader_get_fingerprint(assigned_addr, fingerprint, 0, 32);
            if (ret == 0) {  // COMMAND_OK
                fp_ok = 1;
                break;
            }
        }

        if (!fp_ok) {
            // Fingerprint not ready, release and skip
            pca9557_set_output(0x00);
            delay_ms(10);
            continue;
        }

        // 3f: Start application with matching salt + fingerprint
        ret = bootloader_start_app(assigned_addr, salt, fingerprint);
        if (ret == 0) {
            // Success! Dwarf transitions to application (GREEN LED)
            modbus.discovered_dwarfs |= (1 << dwarf);
        }

        // 3g: Release all from reset for next Dwarf iteration
        pca9557_set_output(0x00);
        delay_ms(10);
    }

    modbus.dwarfs_booted = 1;
}

/****************************************************************
 * Side Filament Sensors - ADC3 with analog mux (Prusa sandwich board)
 *
 * Hardware: 5 filament sensors multiplexed onto 2 ADC3 channels
 * via 2 GPIO select pins. Matches Prusa XLBuddy hardware exactly.
 *
 * Mux positions (setA=PF12, setB=PG6):
 *   pos 0 (A=0,B=0): X(PC0)=sfs1(T0), Y(PF6)=sfs2(T1)
 *   pos 1 (A=1,B=0): X(PC0)=sfs3(T2), Y(PF6)=sfs4(unused)
 *   pos 2 (A=0,B=1): X(PC0)=sfs5(T4), Y(PF6)=sfs6(T3)
 ****************************************************************/

// Side sensor latest readings (12-bit ADC, 0-4095)
static uint16_t side_fs_values[5] = {0};

static void side_fs_init(void)
{
    // Enable GPIO clocks for mux select and ADC pins
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOFEN
                   | RCC_AHB1ENR_GPIOGEN;
    // Enable ADC3 clock (APB2)
    RCC->APB2ENR |= RCC_APB2ENR_ADC3EN;
    for (volatile int i = 0; i < 1000; i++) {}

    // Configure mux select pins as push-pull outputs
    // PF12 (setA)
    SFS_MUX_A_PORT->MODER &= ~(3 << (SFS_MUX_A_PIN * 2));
    SFS_MUX_A_PORT->MODER |= (1 << (SFS_MUX_A_PIN * 2));   // Output
    SFS_MUX_A_PORT->OTYPER &= ~(1 << SFS_MUX_A_PIN);       // Push-pull
    SFS_MUX_A_PORT->BSRR = (1 << (SFS_MUX_A_PIN + 16));    // Start LOW
    // PG6 (setB)
    SFS_MUX_B_PORT->MODER &= ~(3 << (SFS_MUX_B_PIN * 2));
    SFS_MUX_B_PORT->MODER |= (1 << (SFS_MUX_B_PIN * 2));   // Output
    SFS_MUX_B_PORT->OTYPER &= ~(1 << SFS_MUX_B_PIN);       // Push-pull
    SFS_MUX_B_PORT->BSRR = (1 << (SFS_MUX_B_PIN + 16));    // Start LOW

    // Configure ADC input pins as analog mode
    // PC0 = ADC3_IN10
    GPIOC->MODER |= (3 << (0 * 2));    // Analog mode (0b11)
    // PF6 = ADC3_IN4
    GPIOF->MODER |= (3 << (6 * 2));    // Analog mode (0b11)

    // Configure ADC3
    ADC3->CR1 = 0;                      // 12-bit resolution, no scan
    ADC3->CR2 = ADC_CR2_ADON;           // Enable ADC
    ADC3->SMPR1 = 0x07 << (0 * 3);     // CH10: 480 cycles sample time
    ADC3->SMPR2 = 0x07 << (4 * 3);     // CH4: 480 cycles sample time
    ADC3->SQR1 = 0;                     // 1 conversion in sequence
}

// Read a single ADC3 channel (blocking, ~15us)
static uint16_t side_fs_read_channel(uint8_t channel)
{
    ADC3->SQR3 = channel;               // Select channel
    ADC3->CR2 |= ADC_CR2_SWSTART;       // Start conversion
    while (!(ADC3->SR & ADC_SR_EOC)) {}  // Wait for completion
    return (uint16_t)(ADC3->DR & 0xFFF); // Read 12-bit result
}

// Set mux position and read both X and Y channels
static void side_fs_read_mux_pos(uint8_t setA, uint8_t setB,
                                  uint16_t *val_x, uint16_t *val_y)
{
    // Set mux select pins
    if (setA)
        SFS_MUX_A_PORT->BSRR = (1 << SFS_MUX_A_PIN);       // HIGH
    else
        SFS_MUX_A_PORT->BSRR = (1 << (SFS_MUX_A_PIN + 16)); // LOW
    if (setB)
        SFS_MUX_B_PORT->BSRR = (1 << SFS_MUX_B_PIN);       // HIGH
    else
        SFS_MUX_B_PORT->BSRR = (1 << (SFS_MUX_B_PIN + 16)); // LOW

    // Wait for mux to settle (~10us)
    for (volatile int i = 0; i < 200; i++) {}

    // Read both channels
    *val_x = side_fs_read_channel(SFS_ADC_X_CHANNEL);
    *val_y = side_fs_read_channel(SFS_ADC_Y_CHANNEL);
}

// Read all 5 side filament sensors via mux cycling
static void side_fs_read_all(void)
{
    uint16_t x, y;

    // Pos 0 (A=0,B=0): X=sfs1(T0), Y=sfs5(T4)
    side_fs_read_mux_pos(0, 0, &x, &y);
    side_fs_values[0] = x;  // T0 = sfs1
    side_fs_values[4] = y;  // T4 = sfs5

    // Pos 1 (A=1,B=0): X=sfs2(T1), Y=sfs6(T3)
    side_fs_read_mux_pos(1, 0, &x, &y);
    side_fs_values[1] = x;  // T1 = sfs2
    side_fs_values[3] = y;  // T3 = sfs6

    // Pos 2 (A=0,B=1): X=sfs3(T2), Y=sandwich_temp(unused)
    side_fs_read_mux_pos(0, 1, &x, &y);
    side_fs_values[2] = x;  // T2 = sfs3

    // Return mux to pos 0
    SFS_MUX_A_PORT->BSRR = (1 << (SFS_MUX_A_PIN + 16));  // A=LOW
    SFS_MUX_B_PORT->BSRR = (1 << (SFS_MUX_B_PIN + 16));  // B=LOW
}

// Klipper command: query all 5 side filament sensors
void command_query_side_filament_sensors(uint32_t *args)
{
    side_fs_read_all();
    sendf("side_filament_sensors_result s0=%u s1=%u s2=%u s3=%u s4=%u",
          side_fs_values[0], side_fs_values[1], side_fs_values[2],
          side_fs_values[3], side_fs_values[4]);
}
DECL_COMMAND(command_query_side_filament_sensors,
             "query_side_filament_sensors");

/****************************************************************
 * Early Init - Runs at MCU power-on before Klipper connects
 ****************************************************************/

static uint8_t early_init_done = 0;

// Raw register write without going through pca9557_write
static void pca9557_raw_write(uint8_t reg, uint8_t value)
{
    i2c_start();
    i2c_write_byte(0x32);  // PCA9557 write address (0x19 << 1)
    i2c_write_byte(reg);
    i2c_write_byte(value);
    i2c_stop();
}

void
modbus_early_init(void)
{
    // Enable GPIO clocks first
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOFEN;  // For I2C
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOGEN;
    
    // Delay for clocks
    for (volatile int i = 0; i < 20000; i++) {}
    
    // PE7 = GPIOReset - Prusa sets this HIGH (open-drain)
    // This might be a master reset or enable for the I2C expander
    // Configure PE7 as open-drain output, set HIGH
    GPIOE->OTYPER |= (1 << 7);  // Open-drain
    GPIOE->MODER = (GPIOE->MODER & ~(3 << (7 * 2))) | (1 << (7 * 2));  // Output mode
    GPIOE->BSRR = (1 << 7);  // Set HIGH
    
    // PE14 = splitter5vEnable - powers Dwarfs 3-6 on splitter board
    // Configure PE14 as output, set HIGH to enable 5V to splitter
    GPIOE->MODER = (GPIOE->MODER & ~(3 << (14 * 2))) | (1 << (14 * 2));  // Output mode
    GPIOE->BSRR = (1 << 14);  // Set HIGH to enable splitter 5V
    
    // Small delay after enabling power
    for (volatile int i = 0; i < 10000; i++) {}

    // Initialize side filament sensor ADC3 + mux
    side_fs_init();

    // Initialize hardware I2C2
    hw_i2c_init();
    
    // PCA9557 I2C expander controls Dwarf reset pins
    // Pin mapping: p0=dwarf6, p1=dwarf1, p2=dwarf2, p3=dwarf3, p4=dwarf4, p5=dwarf5
    // p6=fanPowerSwitch, p7=modularBedReset
    // LOW = release from reset, HIGH = hold in reset

    // Release all Dwarfs from reset so they boot into bootloader (BLUE LED)
    // The full boot protocol (puppy_boot_sequence) will run later from config_modbus
    // and transition them to application mode (GREEN LED)
    hw_i2c_write(PCA9557_ADDR, PCA9557_REG_OUTPUT, 0x00);
    for (volatile int i = 0; i < 5000; i++) {}
    // Configure all pins as outputs (0 = output)
    hw_i2c_write(PCA9557_ADDR, PCA9557_REG_CONFIG, 0x00);
    
    // Mark early init done
    early_init_done = 1;
    modbus.dwarfs_booted = 0;
}
DECL_INIT(modbus_early_init);

/****************************************************************
 * Klipper Commands
 ****************************************************************/

DECL_CONSTANT("HAVE_MODBUS_MASTER", 1);

void command_config_modbus(uint32_t *args)
{
    // Just init UART - boot sequence is driven from Python host
    modbus_uart_init();
}
DECL_COMMAND(command_config_modbus, "config_modbus");

// Set PCA9557 output register (controls Dwarf reset pins)
// Called from Python to isolate Dwarfs during boot sequence
void command_pca9557_output(uint32_t *args)
{
    uint8_t value = args[0];
    int ret = hw_i2c_write(PCA9557_ADDR, PCA9557_REG_OUTPUT, value);
    sendf("pca9557_output_response status=%c value=%c", ret == 0 ? 0 : 1, value);
}
DECL_COMMAND(command_pca9557_output, "pca9557_output value=%c");

// Send raw bytes and receive response
void command_modbus_send_raw(uint32_t *args)
{
    uint8_t data_len = args[0];
    uint8_t *data = command_decode_ptr(args[1]);

    uint8_t response[128];
    uint16_t resp_len = 0;

    int ret = send_frame_wait(data, data_len, response, &resp_len, sizeof(response), MODBUS_RESPONSE_TIMEOUT_MS);

    // Cap response to fit within MESSAGE_MAX (64 bytes)
    uint16_t send_len = resp_len;
    if (send_len > 50) send_len = 50;
    sendf("modbus_raw_response status=%c data=%*s", ret, send_len, response);
}
DECL_COMMAND(command_modbus_send_raw, "modbus_send_raw data=%*s");

// Read loadcell via FIFO - parses on MCU, returns compact result
void command_modbus_read_loadcell(uint32_t *args)
{
    uint8_t dwarf_addr = args[0];  // MODBUS address (0x1A+)

    // Build FC 0x18 frame: [addr][0x18][0x00][0x00][CRC]
    uint8_t frame[6];
    frame[0] = dwarf_addr;
    frame[1] = 0x18;  // Read FIFO Queue
    frame[2] = 0x00;  // FIFO address high
    frame[3] = 0x00;  // FIFO address low
    uint16_t crc = calc_crc16(frame, 4);
    frame[4] = crc & 0xFF;
    frame[5] = (crc >> 8) & 0xFF;

    uint8_t response[128];
    uint16_t resp_len = 0;

    int ret = send_frame_wait(frame, 6, response, &resp_len, sizeof(response), MODBUS_RESPONSE_TIMEOUT_MS);

    if (ret != MODBUS_OK || resp_len < 8) {
        // No valid response - return error with raw length for debug
        sendf("modbus_loadcell_response status=%c count=%c raw=%u rlen=%u",
              ret, 0, 0, resp_len);
        return;
    }

    // Parse FC 0x18 response: [addr][0x18][bc_hi][bc_lo][fc_hi][fc_lo][data...][CRC]
    // Data starts at offset 6 (after addr+func+bc+fc)
    uint16_t fifo_count = (response[4] << 8) | response[5];

    // FIFO data starts at offset 6, packed into 16-bit MODBUS registers.
    // The Dwarf encoder stores bytes into a uint16_t array using memcpy (LE order),
    // but MODBUS transmits each register as big-endian (MSB first).
    // This means each pair of bytes arrives swapped. We must swap them back.
    uint8_t *fifo_data = &response[6];
    uint16_t fifo_bytes = fifo_count * 2;  // Convert registers to bytes

    // Bounds check: ensure fifo_bytes doesn't exceed actual response buffer
    // Response format: [addr:1][func:1][bc:2][fc:2][data:N][crc:2] = 8 + N bytes
    uint16_t max_fifo_bytes = (resp_len > 8) ? (resp_len - 8) : 0;
    if (fifo_bytes > max_fifo_bytes) {
        fifo_bytes = max_fifo_bytes;
    }

    // Byte-swap each register pair to restore original encoder byte order
    for (uint16_t i = 0; i + 1 < fifo_bytes; i += 2) {
        uint8_t tmp = fifo_data[i];
        fifo_data[i] = fifo_data[i + 1];
        fifo_data[i + 1] = tmp;
    }

    // Now scan for loadcell messages (type 0x02)
    // Each message: [type:1][timestamp:4][raw_value:4] = 9 bytes
    uint32_t last_raw = 0;
    uint8_t sample_count = 0;

    uint16_t pos = 0;
    while (pos < fifo_bytes) {
        uint8_t msg_type = fifo_data[pos];
        if (msg_type == 0x02 && pos + 9 <= fifo_bytes) {
            // Loadcell record: type(1) + timestamp(4) + raw_value(4)
            // Values are little-endian within the byte stream
            last_raw = fifo_data[pos + 5]
                     | (fifo_data[pos + 6] << 8)
                     | (fifo_data[pos + 7] << 16)
                     | (fifo_data[pos + 8] << 24);
            sample_count++;
            pos += 9;
        } else if (msg_type == 0x00) {
            // Padding byte
            pos++;
        } else if (msg_type == 0x01 || msg_type == 0x04) {
            // Log (9 bytes) or accelerometer (9 bytes)
            pos += 9;
        } else if (msg_type == 0x05) {
            // Accelerometer rate (5 bytes)
            pos += 5;
        } else {
            // Unknown type, skip
            pos++;
        }
    }

    sendf("modbus_loadcell_response status=%c count=%c raw=%u rlen=%u",
          ret, sample_count, last_raw, resp_len);
}
DECL_COMMAND(command_modbus_read_loadcell, "modbus_read_loadcell addr=%c");

void command_modbus_status(uint32_t *args)
{
    sendf("modbus_status_response tx=%u rx=%u timeout=%u crc_err=%u init=%c booted=%c dwarfs=%c rx_bytes=%u rx_errs=%u s0=%u s1=%u maxpos=%u",
          modbus.tx_count, modbus.rx_count,
          modbus.timeout_count, modbus.crc_error_count,
          modbus.initialized, modbus.dwarfs_booted, modbus.discovered_dwarfs,
          modbus.rx_total_bytes, modbus.rx_errors,
          modbus.rx_state0_bytes, modbus.rx_state1_bytes, modbus.rx_max_pos);
}
DECL_COMMAND(command_modbus_status, "modbus_status");

// Manual puppy boot command
void command_puppy_boot(uint32_t *args)
{
    puppy_boot_sequence();
    sendf("puppy_boot_response booted=%c dwarfs=%c", modbus.dwarfs_booted, modbus.discovered_dwarfs);
}
DECL_COMMAND(command_puppy_boot, "puppy_boot");

/****************************************************************
 * Task - Check for response timeout
 ****************************************************************/

void modbus_task(void)
{
    if (modbus.state == 2) {  // Receiving
        uint32_t now = timer_read_time();
        if (now - modbus.last_activity > timer_from_us(MODBUS_BYTE_TIMEOUT_US)) {
            modbus.state = 3;
        }
    }
}
DECL_TASK(modbus_task);

/****************************************************************
 * Loadcell Probe State and Commands
 *
 * Implements real-time loadcell monitoring for probe moves.
 * Polls the Dwarf FIFO for loadcell samples and triggers when
 * the threshold is exceeded.
 *
 * Thresholds (from Prusa):
 *   - Static tare: -125g (Z probing)
 *   - Filtered: -40g (continuous)
 *   - XY probe: +40g (dock detection)
 *
 * Scale: raw_adc * 0.0192 = grams
 ****************************************************************/

static struct {
    uint8_t dwarf_addr;          // MODBUS address of active Dwarf
    int32_t threshold_raw;       // Trigger threshold in raw ADC units (signed)
    int32_t tare_offset;         // Tare baseline offset
    int32_t last_load;           // Most recent load reading (tared)
    int32_t min_load;            // Minimum load seen (most negative = most compression)
    int32_t max_load;            // Maximum load seen (most positive = most tension)
    uint8_t triggered;           // Has threshold been crossed
    uint32_t trigger_time;       // MCU clock when triggered
    int32_t trigger_load;        // Load value at trigger
    uint32_t sample_count;       // Total samples processed
    uint8_t monitoring;          // Active monitoring flag
    uint32_t last_poll_time;     // For rate limiting
    uint32_t poll_errors;        // MODBUS communication errors during monitoring
    uint8_t halt_on_trigger;     // If 1, call sched_shutdown() on trigger
    uint8_t xy_mode;             // 0 = Z mode (negative threshold), 1 = XY mode (absolute value)
    struct trsync *ts;           // trsync for endstop-style triggering
    uint8_t trigger_reason;      // Reason code for trsync trigger
    uint8_t homing;              // Currently in homing/probing mode
    uint16_t settle_remaining;   // Samples to skip before trigger detection (larger for XY settling)
} loadcell_probe = {0};

// Poll interval: 3ms matches Prusa's timing, allows ~10 samples per poll at 320Hz
#define LOADCELL_POLL_INTERVAL_US  3000

// Safety: abort if no samples for this many microseconds
#define LOADCELL_STALE_TIMEOUT_US  50000

// Helper: poll FIFO and process loadcell samples
static int loadcell_poll_fifo(void)
{
    // Build FIFO read request (FC 0x18)
    uint8_t frame[6];
    frame[0] = loadcell_probe.dwarf_addr;
    frame[1] = 0x18;  // Read FIFO Queue
    frame[2] = 0x00;
    frame[3] = 0x00;
    uint16_t crc = calc_crc16(frame, 4);
    frame[4] = crc & 0xFF;
    frame[5] = (crc >> 8) & 0xFF;

    uint8_t response[128];
    uint16_t resp_len = 0;

    int ret = send_frame_wait(frame, 6, response, &resp_len,
                              sizeof(response), MODBUS_RESPONSE_TIMEOUT_MS);

    if (ret != MODBUS_OK || resp_len < 8) {
        loadcell_probe.poll_errors++;
        return -1;
    }

    // Parse FIFO response: [addr][0x18][bc_hi][bc_lo][fc_hi][fc_lo][data...][CRC]
    uint16_t fifo_count = (response[4] << 8) | response[5];
    uint8_t *fifo_data = &response[6];
    uint16_t fifo_bytes = fifo_count * 2;

    // Bounds check: ensure fifo_bytes doesn't exceed actual response buffer
    uint16_t max_fifo_bytes = (resp_len > 8) ? (resp_len - 8) : 0;
    if (fifo_bytes > max_fifo_bytes) {
        fifo_bytes = max_fifo_bytes;
    }

    // Byte-swap each register (MODBUS is big-endian, data is little-endian)
    for (uint16_t i = 0; i + 1 < fifo_bytes; i += 2) {
        uint8_t tmp = fifo_data[i];
        fifo_data[i] = fifo_data[i + 1];
        fifo_data[i + 1] = tmp;
    }

    int samples_found = 0;
    uint32_t now = timer_read_time();

    // Process loadcell samples (type 0x02)
    uint16_t pos = 0;
    while (pos < fifo_bytes) {
        uint8_t msg_type = fifo_data[pos];

        if (msg_type == 0x02 && pos + 9 <= fifo_bytes) {
            // Loadcell record: type(1) + timestamp(4) + raw_value(4)
            int32_t raw = (int32_t)(fifo_data[pos + 5]
                        | (fifo_data[pos + 6] << 8)
                        | (fifo_data[pos + 7] << 16)
                        | (fifo_data[pos + 8] << 24));

            // Apply tare offset
            int32_t load = raw - loadcell_probe.tare_offset;
            loadcell_probe.last_load = load;
            loadcell_probe.sample_count++;
            samples_found++;

            // Track min/max loads
            if (load < loadcell_probe.min_load)
                loadcell_probe.min_load = load;
            if (load > loadcell_probe.max_load)
                loadcell_probe.max_load = load;

            // Skip samples during settling period (flush stale FIFO data)
            if (loadcell_probe.settle_remaining > 0) {
                loadcell_probe.settle_remaining--;
                pos += 9;
                continue;
            }

            // Check threshold based on mode
            if (!loadcell_probe.triggered) {
                int trigger = 0;
                if (loadcell_probe.xy_mode) {
                    // XY mode: trigger on absolute value exceeding threshold
                    // Used for tool calibration (hitting pin from side = tension)
                    int32_t abs_load = (load < 0) ? -load : load;
                    int32_t abs_thresh = (loadcell_probe.threshold_raw < 0) ?
                                         -loadcell_probe.threshold_raw :
                                         loadcell_probe.threshold_raw;
                    if (abs_load > abs_thresh) {
                        trigger = 1;
                    }
                } else {
                    // Z mode: trigger on negative load (compression)
                    // threshold_raw is negative, load must be more negative
                    if (load < loadcell_probe.threshold_raw) {
                        trigger = 1;
                    }
                }
                if (trigger) {
                    loadcell_probe.triggered = 1;
                    loadcell_probe.trigger_time = now;
                    loadcell_probe.trigger_load = load;
                    // Trigger trsync to stop motion (like endstop)
                    if (loadcell_probe.homing && loadcell_probe.ts) {
                        trsync_do_trigger(loadcell_probe.ts, loadcell_probe.trigger_reason);
                        loadcell_probe.homing = 0;  // One-shot
                    }
                    // Emergency stop if requested (fallback)
                    if (loadcell_probe.halt_on_trigger) {
                        shutdown("Loadcell probe triggered");
                    }
                }
            }

            pos += 9;
        } else if (msg_type == 0x00) {
            pos++;  // Padding byte
        } else if (msg_type == 0x01 || msg_type == 0x04) {
            pos += 9;  // Log (9 bytes) or accelerometer (9 bytes)
        } else if (msg_type == 0x05) {
            pos += 5;  // Accelerometer rate (5 bytes)
        } else {
            pos++;  // Unknown, skip
        }
    }

    return samples_found;
}

// Configure loadcell probe parameters
void command_loadcell_probe_config(uint32_t *args)
{
    loadcell_probe.dwarf_addr = args[0];
    loadcell_probe.threshold_raw = (int32_t)args[1];

    sendf("loadcell_probe_config_ack addr=%c threshold=%i",
          loadcell_probe.dwarf_addr, loadcell_probe.threshold_raw);
}
DECL_COMMAND(command_loadcell_probe_config,
             "loadcell_probe_config addr=%c threshold=%i");

// Start loadcell probe monitoring with optional emergency halt
// xy_mode: 0 = Z mode (negative threshold, compression), 1 = XY mode (absolute value, tension)
void command_loadcell_probe_start(uint32_t *args)
{
    loadcell_probe.dwarf_addr = args[0];
    loadcell_probe.threshold_raw = (int32_t)args[1];
    loadcell_probe.halt_on_trigger = args[2];  // 1 = emergency stop on trigger
    loadcell_probe.xy_mode = args[3];          // 0 = Z mode, 1 = XY mode

    // Reset state
    loadcell_probe.triggered = 0;
    loadcell_probe.trigger_time = 0;
    loadcell_probe.trigger_load = 0;
    loadcell_probe.sample_count = 0;
    loadcell_probe.last_load = 0;
    loadcell_probe.min_load = 0;
    loadcell_probe.max_load = 0;
    loadcell_probe.poll_errors = 0;
    loadcell_probe.last_poll_time = timer_read_time();

    // Enable monitoring
    loadcell_probe.monitoring = 1;

    sendf("loadcell_probe_start_ack addr=%c threshold=%i halt=%c xy=%c",
          loadcell_probe.dwarf_addr, loadcell_probe.threshold_raw,
          loadcell_probe.halt_on_trigger, loadcell_probe.xy_mode);
}
DECL_COMMAND(command_loadcell_probe_start,
             "loadcell_probe_start addr=%c threshold=%i halt=%c xy=%c");

// Stop loadcell probe monitoring and return results
void command_loadcell_probe_stop(uint32_t *args)
{
    loadcell_probe.monitoring = 0;

    sendf("loadcell_probe_result triggered=%c load=%i time=%u count=%u min=%i max=%i errors=%u xy=%c",
          loadcell_probe.triggered,
          loadcell_probe.last_load,
          loadcell_probe.trigger_time,
          loadcell_probe.sample_count,
          loadcell_probe.min_load,
          loadcell_probe.max_load,
          loadcell_probe.poll_errors,
          loadcell_probe.xy_mode);
}
DECL_COMMAND(command_loadcell_probe_stop, "loadcell_probe_stop");

// Query current probe state without stopping
void command_loadcell_probe_query(uint32_t *args)
{
    sendf("loadcell_probe_state triggered=%c load=%i count=%u monitoring=%c",
          loadcell_probe.triggered,
          loadcell_probe.last_load,
          loadcell_probe.sample_count,
          loadcell_probe.monitoring);
}
DECL_COMMAND(command_loadcell_probe_query, "loadcell_probe_query");

// Setup loadcell as endstop for probing (like endstop_home)
// This makes the loadcell trigger trsync when threshold exceeded
void command_loadcell_endstop_home(uint32_t *args)
{
    loadcell_probe.dwarf_addr = args[0];
    loadcell_probe.threshold_raw = (int32_t)args[1];
    loadcell_probe.xy_mode = args[2];

    uint8_t trsync_oid = args[3];
    uint8_t trigger_reason = args[4];

    // Reset state
    loadcell_probe.triggered = 0;
    loadcell_probe.trigger_time = 0;
    loadcell_probe.trigger_load = 0;
    loadcell_probe.sample_count = 0;
    loadcell_probe.last_load = 0;
    loadcell_probe.min_load = 0;
    loadcell_probe.max_load = 0;
    loadcell_probe.poll_errors = 0;
    loadcell_probe.halt_on_trigger = 0;
    loadcell_probe.last_poll_time = timer_read_time();
    loadcell_probe.settle_remaining = 32;   // Skip first 32 samples (~100ms) to flush stale FIFO data

    // Setup trsync for endstop-style triggering
    if (trsync_oid == 0xff) {
        // Disable homing
        loadcell_probe.ts = NULL;
        loadcell_probe.homing = 0;
        loadcell_probe.monitoring = 0;
    } else {
        loadcell_probe.ts = trsync_oid_lookup(trsync_oid);
        loadcell_probe.trigger_reason = trigger_reason;
        loadcell_probe.homing = 1;
        loadcell_probe.monitoring = 1;
    }

    sendf("loadcell_endstop_home_ack addr=%c threshold=%i xy=%c trsync=%c reason=%c",
          loadcell_probe.dwarf_addr, loadcell_probe.threshold_raw,
          loadcell_probe.xy_mode, trsync_oid, trigger_reason);
}
DECL_COMMAND(command_loadcell_endstop_home,
             "loadcell_endstop_home addr=%c threshold=%i xy=%c trsync_oid=%c trigger_reason=%c");

// Tare the loadcell (zero offset on current load)
void command_loadcell_tare(uint32_t *args)
{
    uint8_t addr = args[0];
    uint8_t num_samples = args[1];
    if (num_samples > 64) num_samples = 64;
    if (num_samples < 8) num_samples = 8;

    // Save current state
    uint8_t was_monitoring = loadcell_probe.monitoring;
    loadcell_probe.monitoring = 0;

    int32_t sum = 0;
    int count = 0;

    // Collect samples over multiple polls
    loadcell_probe.dwarf_addr = addr;
    for (int polls = 0; polls < 10 && count < num_samples; polls++) {
        delay_ms(5);  // Wait for FIFO to fill

        // Build FIFO read request
        uint8_t frame[6];
        frame[0] = addr;
        frame[1] = 0x18;
        frame[2] = 0x00;
        frame[3] = 0x00;
        uint16_t crc = calc_crc16(frame, 4);
        frame[4] = crc & 0xFF;
        frame[5] = (crc >> 8) & 0xFF;

        uint8_t response[128];
        uint16_t resp_len = 0;

        int ret = send_frame_wait(frame, 6, response, &resp_len,
                                  sizeof(response), MODBUS_RESPONSE_TIMEOUT_MS);

        if (ret != MODBUS_OK || resp_len < 8)
            continue;

        // Parse response
        uint16_t fifo_count = (response[4] << 8) | response[5];
        uint8_t *fifo_data = &response[6];
        uint16_t fifo_bytes = fifo_count * 2;

        // Bounds check: ensure fifo_bytes doesn't exceed actual response buffer
        uint16_t max_fifo_bytes = (resp_len > 8) ? (resp_len - 8) : 0;
        if (fifo_bytes > max_fifo_bytes) {
            fifo_bytes = max_fifo_bytes;
        }

        // Byte-swap
        for (uint16_t i = 0; i + 1 < fifo_bytes; i += 2) {
            uint8_t tmp = fifo_data[i];
            fifo_data[i] = fifo_data[i + 1];
            fifo_data[i + 1] = tmp;
        }

        // Extract loadcell samples
        uint16_t pos = 0;
        while (pos < fifo_bytes && count < num_samples) {
            if (fifo_data[pos] == 0x02 && pos + 9 <= fifo_bytes) {
                int32_t raw = (int32_t)(fifo_data[pos + 5]
                            | (fifo_data[pos + 6] << 8)
                            | (fifo_data[pos + 7] << 16)
                            | (fifo_data[pos + 8] << 24));
                sum += raw;
                count++;
                pos += 9;
            } else if (fifo_data[pos] == 0x00) {
                pos++;
            } else if (fifo_data[pos] == 0x01 || fifo_data[pos] == 0x04) {
                pos += 9;
            } else if (fifo_data[pos] == 0x05) {
                pos += 5;
            } else {
                pos++;
            }
        }
    }

    // Calculate average as tare offset
    if (count > 0) {
        loadcell_probe.tare_offset = sum / count;
    }

    // Restore monitoring state
    loadcell_probe.monitoring = was_monitoring;

    sendf("loadcell_tare_result offset=%i samples=%c",
          loadcell_probe.tare_offset, count);
}
DECL_COMMAND(command_loadcell_tare, "loadcell_tare addr=%c samples=%c");

// Loadcell probe polling task - runs continuously when monitoring is active
void loadcell_probe_task(void)
{
    if (!loadcell_probe.monitoring)
        return;

    // Already triggered - stop polling
    if (loadcell_probe.triggered)
        return;

    // Rate limit polling
    uint32_t now = timer_read_time();
    if (now - loadcell_probe.last_poll_time < timer_from_us(LOADCELL_POLL_INTERVAL_US))
        return;
    loadcell_probe.last_poll_time = now;

    // Poll the FIFO
    loadcell_poll_fifo();
}
DECL_TASK(loadcell_probe_task);

/****************************************************************
 * Filament Sensor Endstop
 *
 * Reads tool filament sensor ADC (register 0x8063) via MODBUS
 * FC 0x04 (read input registers) and triggers trsync when the
 * ADC value crosses the midpoint threshold indicating filament
 * is present. Used for autoload assist insertion.
 *
 * Modeled on loadcell endstop but much simpler - single register
 * read instead of FIFO parsing.
 ****************************************************************/

// Poll interval: 50ms (20Hz) - filament detection doesn't need high speed
#define FILAMENT_POLL_INTERVAL_US  50000

static struct {
    uint8_t dwarf_addr;          // MODBUS address of Dwarf to monitor
    uint16_t threshold;          // ADC midpoint threshold
    uint8_t ins_above;           // 1 if inserted value > nins value
    uint16_t last_adc;           // Most recent ADC reading
    uint8_t triggered;           // Has filament been detected
    uint32_t trigger_time;       // MCU clock when triggered
    uint8_t monitoring;          // Active monitoring flag
    uint32_t last_poll_time;     // For rate limiting
    uint32_t poll_count;         // Total polls performed
    uint32_t poll_errors;        // MODBUS communication errors
    uint8_t consec_detect;       // Consecutive "has_filament" readings
    struct trsync *ts;           // trsync for endstop-style triggering
    uint8_t trigger_reason;      // Reason code for trsync trigger
    uint8_t homing;              // Currently in homing mode
} filament_endstop = {0};

// Read filament sensor register 0x8063 via FC 0x04
static int filament_read_sensor(uint16_t *adc_out)
{
    // Build FC 0x04 (Read Input Registers) request for register 0x8063, count=1
    uint8_t frame[8];
    frame[0] = filament_endstop.dwarf_addr;
    frame[1] = 0x04;  // FC: Read Input Registers
    frame[2] = 0x80;  // Register high byte
    frame[3] = 0x63;  // Register low byte
    frame[4] = 0x00;  // Count high byte
    frame[5] = 0x01;  // Count low byte
    uint16_t crc = calc_crc16(frame, 6);
    frame[6] = crc & 0xFF;
    frame[7] = (crc >> 8) & 0xFF;

    uint8_t response[32];
    uint16_t resp_len = 0;

    int ret = send_frame_wait(frame, 8, response, &resp_len,
                              sizeof(response), MODBUS_RESPONSE_TIMEOUT_MS);

    if (ret != MODBUS_OK || resp_len < 7) {
        filament_endstop.poll_errors++;
        return -1;
    }

    // FC 0x04 response: [addr][0x04][byte_count][data_hi][data_lo][CRC]
    if (response[1] != 0x04 || response[2] != 2) {
        filament_endstop.poll_errors++;
        return -1;
    }

    *adc_out = (response[3] << 8) | response[4];
    return 0;
}

// Setup filament sensor as endstop for homing_move
void command_filament_endstop_home(uint32_t *args)
{
    filament_endstop.dwarf_addr = args[0];
    filament_endstop.threshold = args[1];
    filament_endstop.ins_above = args[2];

    uint8_t trsync_oid = args[3];
    uint8_t trigger_reason = args[4];

    // Reset state
    filament_endstop.triggered = 0;
    filament_endstop.trigger_time = 0;
    filament_endstop.last_adc = 0;
    filament_endstop.poll_count = 0;
    filament_endstop.poll_errors = 0;
    filament_endstop.consec_detect = 0;
    filament_endstop.last_poll_time = timer_read_time();

    if (trsync_oid == 0xff) {
        // Disable homing
        filament_endstop.ts = NULL;
        filament_endstop.homing = 0;
        filament_endstop.monitoring = 0;
    } else {
        filament_endstop.ts = trsync_oid_lookup(trsync_oid);
        filament_endstop.trigger_reason = trigger_reason;
        filament_endstop.homing = 1;
        filament_endstop.monitoring = 1;
    }

    sendf("filament_endstop_home_ack addr=%c threshold=%hu above=%c trsync=%c",
          filament_endstop.dwarf_addr, filament_endstop.threshold,
          filament_endstop.ins_above, trsync_oid);
}
DECL_COMMAND(command_filament_endstop_home,
             "filament_endstop_home addr=%c threshold=%hu above=%c trsync_oid=%c trigger_reason=%c");

// Filament sensor polling task
void filament_sensor_task(void)
{
    if (!filament_endstop.monitoring)
        return;

    if (filament_endstop.triggered)
        return;

    // Rate limit polling
    uint32_t now = timer_read_time();
    if (now - filament_endstop.last_poll_time < timer_from_us(FILAMENT_POLL_INTERVAL_US))
        return;
    filament_endstop.last_poll_time = now;

    // Read sensor
    uint16_t adc = 0;
    if (filament_read_sensor(&adc) != 0)
        return;

    filament_endstop.last_adc = adc;
    filament_endstop.poll_count++;

    // Check if filament detected: ADC crossed threshold
    // ins_above=1: filament present when adc > threshold
    // ins_above=0: filament present when adc < threshold
    int detected = 0;
    if (filament_endstop.ins_above) {
        if (adc > filament_endstop.threshold)
            detected = 1;
    } else {
        if (adc < filament_endstop.threshold)
            detected = 1;
    }

    if (detected) {
        filament_endstop.consec_detect++;
        // Require 2 consecutive detections to filter noise
        if (filament_endstop.consec_detect >= 2) {
            filament_endstop.triggered = 1;
            filament_endstop.trigger_time = now;
            if (filament_endstop.homing && filament_endstop.ts) {
                trsync_do_trigger(filament_endstop.ts,
                                  filament_endstop.trigger_reason);
                filament_endstop.homing = 0;  // One-shot
            }
        }
    } else {
        filament_endstop.consec_detect = 0;
    }
}
DECL_TASK(filament_sensor_task);
