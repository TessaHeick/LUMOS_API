#ifndef NAND_DRIVER_H
#define NAND_DRIVER_H

#include <stdint.h>
#include <Arduino.h>

// Address structure
struct nand_address {
    unsigned int column : 13;
    unsigned int page   : 6;
    unsigned int block  : 11;
    unsigned int        : 2; // padding
};

extern nand_address flashAddr;

// --- Function Declarations ---

bool begin(); // Initialize NAND driver
void reset_chip(); // Reset the NAND chip
bool read_ID(uint8_t &mfg, uint8_t &dev); // Read manufacturer and device ID
uint8_t get_status(); // Get status register
int write_bytes(const uint8_t* data, uint16_t length); // Write bytes to NAND
void send_command(uint8_t command); // Send command to NAND
void print_status(uint8_t sr); // Print status register details
void unlock_all_blocks(); // Unlock all blocks
bool erase_block(uint16_t block); // Erase a block
bool read_bytes(uint8_t* out, uint16_t length); // Read bytes from NAND

#endif