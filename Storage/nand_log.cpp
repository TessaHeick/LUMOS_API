#include "nand_log.h"

// Internal pointers/state
static uint16_t g_startBlk, g_endBlk;
static uint16_t g_curBlk;
static uint8_t  g_curPage;
static uint16_t g_curCol;
static uint32_t g_nextSeq;
static bool     g_haveRange = false;
static bool     g_logFull = false; // True when log is full

// True means the block is bad (factory-marked)
static bool g_bad[NAND_BLOCK_COUNT];

// Iterator
static uint16_t it_blk; static uint8_t it_page; static uint16_t it_col;

// Buffer for current payload
static constexpr uint16_t max_payload_len = NAND_MAIN_BYTES - sizeof(LogHdr);
static uint8_t payload[max_payload_len];
static uint16_t payload_len = 0;

// ---------- internal helpers ----------

// Checksum calculation
static inline uint16_t crc16_ccitt(const uint8_t* d, uint16_t n) {
  uint16_t c = 0xFFFF;
  for (uint16_t i=0;i<n;i++) {
    c ^= (uint16_t)d[i] << 8;
    for (uint8_t b=0;b<8;b++)
      c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
  }
  return c;
}

// Change the global flashAddr to point to given blk/page/col
static inline void set_addr(uint16_t blk, uint8_t page, uint16_t col) {
  flashAddr.block  = blk;
  flashAddr.page   = page & 0x3F;
  flashAddr.column = col & 0x1FFF;
}

// Returns true if block is factory-marked bad
static bool is_factory_bad(uint16_t blk) {
  // Convention: first spare byte of page 0 marks a bad block if != 0xFF
  uint8_t mark = 0xFF;
  set_addr(blk, 0, NAND_MAIN_BYTES); // first byte in spare(OOB)
  return (!read_bytes(&mark, 1)) || (mark != 0xFF);
}

// Returns next good block after blk; sets g_logFull if none found
static uint16_t next_good_block(uint16_t blk) {
  uint16_t b = blk;
  while (true) {
    if (++b > g_endBlk) { g_logFull = true; return blk; } // no wrap - set full flag and return current
    if (!g_bad[b]) return b;
  }
}

// Returns true if page looks unused (first two bytes 0xFF)
static bool page_is_blank(uint16_t blk, uint8_t page) {
  uint8_t probe[2] = {0};
  set_addr(blk, page, 0);
  if (!read_bytes(probe, 2)) return false;
  return (probe[0] == 0xFF && probe[1] == 0xFF);
}

// Advance the write cursor by n bytes, crossing page/block boundaries as needed
static void advance_by(uint32_t n) {
  if (g_logFull) return;
  while (n > 0) {
    uint32_t rem = NAND_MAIN_BYTES - g_curCol;
    uint32_t step = (n < rem) ? n : rem;
    g_curCol += (uint16_t)step;
    n -= step;
    if (g_curCol >= NAND_MAIN_BYTES) {
      g_curCol = 0;
      g_curPage++;
      if (g_curPage >= NAND_PAGES_PER_BLOCK) {
        // move to next good block; erase it for fresh programming
        g_curPage = 0;
        g_curBlk = next_good_block(g_curBlk);
        if (g_logFull) return; // reached end, no wrap
        // Only erase if not already blank
        if (!page_is_blank(g_curBlk, 0)) erase_block(g_curBlk);
      }
    }
  }
}

// Set flashAddr to current write cursor
static void cursor_to_flash() { set_addr(g_curBlk, g_curPage, g_curCol); }

// Write one payload record (header + payload) into current page at current cursor
static bool write_one_page_payload(const uint8_t* buf, uint16_t len, uint32_t seq) {
  if (g_logFull) return false;
  // Write [LogHdr][payload] in a single program operation (<= main bytes)
  LogHdr h;
  h.magic   = 0xA55A;
  h.length  = len;
  h.seq     = seq;
  h.crc16   = crc16_ccitt(buf, len);
  // Tmp for header CRC
  uint8_t tmp[sizeof(LogHdr)-2];
  memcpy(tmp, &h, sizeof(LogHdr)-2);
  h.hdr_crc = crc16_ccitt(tmp, sizeof(tmp));

  // Must not cross page boundary
  const uint32_t needed = (uint32_t)sizeof(LogHdr) + len;
  if (g_curCol + needed > NAND_MAIN_BYTES) return false;

  // Program in one shot to avoid multiple partial programs
  cursor_to_flash();
  // Build a small staging copy in chunks (avoid big RAM)
  // 1) header
  write_bytes(reinterpret_cast<const uint8_t*>(&h), sizeof(LogHdr));
  // 2) payload
  cursor_to_flash(); // write_bytes advanced within the page; reset addr for contiguous write
  advance_by(sizeof(LogHdr));
  cursor_to_flash();
  write_bytes(buf, len);

  // advance cursor past this record (and pad to end if desired)
  advance_by(len);
  return true;
}

// Read and validate LogHdr at given location; returns false if invalid
static bool read_hdr_at(uint16_t blk, uint8_t page, uint16_t col, LogHdr& out) {
  set_addr(blk, page, col);
  if (!read_bytes(reinterpret_cast<uint8_t*>(&out), sizeof(LogHdr))) return false;
  if (out.magic != 0xA55A) return false;

  uint8_t tmp[sizeof(LogHdr)-2];
  memcpy(tmp, &out, sizeof(LogHdr)-2);
  uint16_t expect = crc16_ccitt(tmp, sizeof(tmp));
  return expect == out.hdr_crc;
}

// ---------- public API ----------

// Initialize logging over given block range (inclusive); optionally format if blank
bool log_begin(uint16_t start_block, uint16_t end_block, bool format_if_blank) {
  if (start_block > end_block) return false;
  if (end_block >= NAND_BLOCK_COUNT || start_block >= NAND_BLOCK_COUNT) return false;
  if (start_block < 0 || end_block < 0) return false;
  g_startBlk = start_block;
  g_endBlk   = end_block;
  g_haveRange = true;

  // Build bad-block table (factory bad); keep runtime-grown bads out of scope for now
  for (uint16_t b = 0; b < NAND_BLOCK_COUNT; ++b) g_bad[b] = false;
  for (uint16_t b = g_startBlk; b <= g_endBlk; ++b) {
    g_bad[b] = is_factory_bad(b);
  }

  // Optionally format (erase) the range
  if (format_if_blank) {
    for (uint16_t b = g_startBlk; b <= g_endBlk; ++b) {
      if (!g_bad[b]) erase_block(b);
    }
  }

  // Find the first non-blank page (oldest) and the first blank page (tip)
  uint16_t tipBlk = g_startBlk; uint8_t tipPage = 0;
  bool found_tip = false;
  uint32_t maxSeq = 0;

  for (uint16_t b = g_startBlk; b <= g_endBlk; ++b) {
    if (g_bad[b]) continue;
    for (uint8_t p = 0; p < NAND_PAGES_PER_BLOCK; ++p) {
      if (page_is_blank(b, p)) { tipBlk = b; tipPage = p; found_tip = true; break; }
      // track max seq to continue counting if needed
      LogHdr h;
      if (read_hdr_at(b, p, 0, h)) {
        if (h.seq > maxSeq) maxSeq = h.seq;
      }
    }
    if (found_tip) break;
  }

  if (!found_tip) {
    // Range completely full; mark as full (no wrap)
    g_logFull = true;
    g_curBlk = g_endBlk;
    g_curPage = NAND_PAGES_PER_BLOCK - 1;
    g_curCol = 0;
  } else {
    g_logFull = false;
    g_curBlk = tipBlk;
    g_curPage = tipPage;
    g_curCol = 0;
  }
  g_nextSeq = maxSeq + 1;

  // Init iterator at oldest page
  it_blk = g_curBlk; it_page = g_curPage; it_col = 0; // will be reset by log_iter_reset()
  return true;
}

// Append data to log (may buffer internally); returns false if log full or error
bool log_append(const uint8_t* data, uint16_t len) {
  if (!g_haveRange || !data || len == 0) return false;
  if (g_logFull) return false; // log is full

  if (payload_len + len <= max_payload_len) {
    // Append to current payload buffer
    memcpy(&payload[payload_len], data, len);
    payload_len += len;
    return true;
  }

  memcpy(&payload[payload_len], data, max_payload_len - payload_len);

  // If record won't fit in remaining bytes of this page, move to next page
  if (g_curCol > 0) {
    // Pad remainder (optional: write 0xFF — but NAND already reads as 0xFF when blank)
    g_curCol = 0;
    g_curPage++;
    if (g_curPage >= NAND_PAGES_PER_BLOCK) {
      g_curPage = 0;
      g_curBlk = next_good_block(g_curBlk);
      if (g_logFull) return false; // Reached end, no wrap

      // Only erase if not already blank
      if (!page_is_blank(g_curBlk, 0)) erase_block(g_curBlk);
    }
  }

  // Write buffered payload
  bool ok = write_one_page_payload(payload, max_payload_len, g_nextSeq++);
  if (!ok) return false;

  // Move to page boundary after record to keep "<= 1 program per page" rule
  if (g_curCol != 0) { g_curCol = 0; g_curPage++; }
  if (g_curPage >= NAND_PAGES_PER_BLOCK) {
    g_curPage = 0;
    g_curBlk = next_good_block(g_curBlk);
    if (g_logFull) {
      // Log is now full
      payload_len = 0;
      return false;
    }
    // Only erase if not already blank
    if (!page_is_blank(g_curBlk, 0)) erase_block(g_curBlk);
  }

  // Reset payload buffer with leftover data
  uint16_t bytes_copied = max_payload_len - payload_len;
  payload_len = len - bytes_copied;
  memcpy(payload, &data[bytes_copied], payload_len);

  return true;
}

// Flush any buffered data to NAND; returns false if log full or error
bool log_flush() {
  if (!g_haveRange || payload_len == 0) return true;
  if (g_logFull) return false; // log is full

  // If record won't fit in remaining bytes of this page, move to next page
  if (g_curCol > 0) {
    g_curCol = 0;
    g_curPage++;
    if (g_curPage >= NAND_PAGES_PER_BLOCK) {
      g_curPage = 0;
      g_curBlk = next_good_block(g_curBlk);
      if (g_logFull) return false; // Reached end, no wrap

      // Only erase if not already blank
      if (!page_is_blank(g_curBlk, 0)) erase_block(g_curBlk);
    }
  }

  // Write buffered payload
  bool ok = write_one_page_payload(payload, payload_len, g_nextSeq++);
  if (!ok) return false;

  // Move to page boundary after record
  if (g_curCol != 0) { g_curCol = 0; g_curPage++; }
  if (g_curPage >= NAND_PAGES_PER_BLOCK) {
    g_curPage = 0;
    g_curBlk = next_good_block(g_curBlk);
    if (g_logFull) {
      // Log is now full
      payload_len = 0;
      return false;
    }
    // Only erase if not already blank
    if (!page_is_blank(g_curBlk, 0)) erase_block(g_curBlk);
  }

  // Clear the buffer
  payload_len = 0;
  return true;
}

// Reset iterator to start of log
void log_iter_reset() {
  // Find first non-blank page
  for (uint16_t b = g_startBlk; b <= g_endBlk; ++b) {
    if (g_bad[b]) continue;
    for (uint8_t p = 0; p < NAND_PAGES_PER_BLOCK; ++p) {
      if (!page_is_blank(b,p)) { it_blk=b; it_page=p; it_col=0; return; }
    }
  }
  it_blk = g_startBlk; it_page = 0; it_col = 0;
}

// Get next log record into out buffer (up to max_out); returns false if end or error
bool log_iter_next(uint8_t* out, uint16_t max_out, uint16_t* out_len) {
  // Stop if we reached the tip (blank page)
  if (page_is_blank(it_blk, it_page)) return false;

  LogHdr h;
  if (!read_hdr_at(it_blk, it_page, 0, h)) {
    // Corrupt header or torn write: treat as end
    return false;
  }
  if (h.length > max_out) return false;

  // Read payload and CRC-check
  set_addr(it_blk, it_page, sizeof(LogHdr));
  if (!read_bytes(out, h.length)) return false;
  if (crc16_ccitt(out, h.length) != h.crc16) return false;

  if (out_len) *out_len = h.length;

  // Advance iterator to next page
  it_page++;
  if (it_page >= NAND_PAGES_PER_BLOCK) {
    it_page = 0;
    it_blk++;
    if (it_blk > g_endBlk) return false; // Reached end, no wrap
  }
  return true;
}

// Erase and format the entire log range
void log_format_range() {
  for (uint16_t b = g_startBlk; b <= g_endBlk; ++b) if (!g_bad[b]) erase_block(b);
  g_curBlk=g_startBlk; g_curPage=0; g_curCol=0; g_nextSeq=1; g_logFull=false;
}

// Get current log record count and next sequence number
uint32_t log_record_count() { return g_nextSeq ? (g_nextSeq - 1) : 0; }

// Get next sequence number to be assigned
uint32_t log_next_seq() { return g_nextSeq; }