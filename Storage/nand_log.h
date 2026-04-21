#pragma once
#include <Arduino.h>
#include <stdint.h>
#include "nand_driver.h"

// --- Configuration ---
#ifndef NAND_MAIN_BYTES
#define NAND_MAIN_BYTES   4096    // main area per page
#endif
#ifndef NAND_SPARE_BYTES
#define NAND_SPARE_BYTES   256    // spare(OOB) per page
#endif
#ifndef NAND_PAGES_PER_BLOCK
#define NAND_PAGES_PER_BLOCK 64  // pages per block
#endif
#ifndef NAND_BLOCK_COUNT
#define NAND_BLOCK_COUNT  2048  // total blocks in device
#endif

// Header structure for each record
struct __attribute__((packed)) LogHdr {
  uint16_t magic; // marker to show valid record
  uint16_t length; // length of payload in bytes
  uint32_t seq; // sequence number
  uint16_t crc16; // payload checksum
  uint16_t hdr_crc; // header checksum
};

bool log_begin(uint16_t start_block, uint16_t end_block, bool format_if_blank);
bool log_append(const uint8_t* data, uint16_t len);
bool log_flush();  // flush any buffered data to flash

// Iterator over log records (oldest to newest)
void log_iter_reset();
bool log_iter_next(uint8_t* out, uint16_t max_out, uint16_t* out_len);

void log_format_range(); // erase all blocks in range
uint32_t log_record_count(); // number of records in log
uint32_t log_next_seq(); // next sequence number to be written