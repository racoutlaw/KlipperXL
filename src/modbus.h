// SPDX-License-Identifier: GPL-3.0-or-later
// Klipper MODBUS Master for Prusa XL
// Copyright (C) 2026 Richard & Claude
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#ifndef __MODBUS_H
#define __MODBUS_H

#include <stdint.h>

/****************************************************************
 * MODBUS Function Codes
 ****************************************************************/

#define MODBUS_FC_READ_DISCRETE_INPUTS    0x02
#define MODBUS_FC_READ_HOLDING_REGISTERS  0x03
#define MODBUS_FC_READ_INPUT_REGISTERS    0x04
#define MODBUS_FC_WRITE_SINGLE_COIL       0x05
#define MODBUS_FC_WRITE_SINGLE_REGISTER   0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGS     0x10
#define MODBUS_FC_READ_FIFO               0x18

/****************************************************************
 * MODBUS Error Codes
 ****************************************************************/

#define MODBUS_OK                         0
#define MODBUS_ERR_TIMEOUT                1
#define MODBUS_ERR_CRC                    2
#define MODBUS_ERR_EXCEPTION              3
#define MODBUS_ERR_FRAME                  4
#define MODBUS_ERR_BUSY                   5

/****************************************************************
 * MODBUS Exception Codes
 ****************************************************************/

#define MODBUS_EX_ILLEGAL_FUNCTION        0x01
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS    0x02
#define MODBUS_EX_ILLEGAL_DATA_VALUE      0x03
#define MODBUS_EX_SLAVE_DEVICE_FAILURE    0x04

/****************************************************************
 * Configuration
 ****************************************************************/

// Maximum data in a single MODBUS frame (excluding address, function, CRC)
#define MODBUS_MAX_DATA_LEN               252

// Response timeout in microseconds
#define MODBUS_RESPONSE_TIMEOUT_US        100000  // 100ms

// Inter-frame gap in microseconds (3.5 character times at 230400 baud)
#define MODBUS_INTER_FRAME_GAP_US         1750

// Maximum number of registers per read
#define MODBUS_MAX_REGISTERS              125

/****************************************************************
 * Data Structures
 ****************************************************************/

struct modbus_master {
    // UART configuration
    struct uart_config *uart;
    
    // RS485 direction pin
    struct gpio_out dir_pin;
    
    // Timing
    uint32_t baud;
    uint32_t last_transaction_time;
    
    // State
    uint8_t busy;
    
    // Buffers
    uint8_t tx_buf[256];
    uint8_t rx_buf[256];
    uint16_t tx_len;
    uint16_t rx_len;
    
    // Statistics
    uint32_t tx_count;
    uint32_t rx_count;
    uint32_t error_count;
    uint32_t timeout_count;
    uint32_t crc_error_count;
};

/****************************************************************
 * Function Prototypes
 ****************************************************************/

// Initialization
struct modbus_master *modbus_master_alloc(void);
void modbus_master_init(struct modbus_master *mb, 
                        uint32_t uart_bus,
                        uint32_t dir_pin, 
                        uint32_t baud);

// Low-level functions
uint16_t modbus_crc16(uint8_t *data, uint16_t len);
int modbus_transaction(struct modbus_master *mb, 
                       uint8_t address,
                       uint8_t *tx_data, uint16_t tx_len,
                       uint8_t *rx_data, uint16_t *rx_len,
                       uint16_t expected_rx_len);

// Read functions
int modbus_read_input_registers(struct modbus_master *mb, 
                                uint8_t address,
                                uint16_t start_reg, 
                                uint16_t count,
                                uint16_t *data);

int modbus_read_holding_registers(struct modbus_master *mb, 
                                  uint8_t address,
                                  uint16_t start_reg, 
                                  uint16_t count,
                                  uint16_t *data);

int modbus_read_discrete_inputs(struct modbus_master *mb, 
                                uint8_t address,
                                uint16_t start, 
                                uint16_t count,
                                uint8_t *data);

// Write functions
int modbus_write_single_register(struct modbus_master *mb, 
                                 uint8_t address,
                                 uint16_t reg, 
                                 uint16_t value);

int modbus_write_multiple_registers(struct modbus_master *mb, 
                                    uint8_t address,
                                    uint16_t start_reg, 
                                    uint16_t count,
                                    uint16_t *data);

int modbus_write_coil(struct modbus_master *mb, 
                      uint8_t address,
                      uint16_t coil, 
                      uint8_t value);

// FIFO (for loadcell/accelerometer)
int modbus_read_fifo(struct modbus_master *mb, 
                     uint8_t address,
                     uint16_t fifo_address,
                     uint16_t *data, 
                     uint16_t *count);

#endif // __MODBUS_H
