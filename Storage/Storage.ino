#include <Wire.h>
#include "Adafruit_TLC5947.h"
#include <Adafruit_AS7341.h>
#include "nand_driver.h"
#include "nand_log.h"
#include "AS7341_subclass.h"

/* ###################     CONTROL FLAGS     ################### */
// Boot-time mode selection.
// If either flag is true, the sketch will run that routine and then halt.
bool readBackBool = false;   // true => run readback() and do nothing else
bool eraseBool    = false;   // true => run erase() and do nothing else
bool serialDebug = true;  // device will serial output all the data, has to be true to receive data, if checkCharging is true then this won't be able to print anything so default by serialDebug (but it can also print detecting the charging state)

// ================= USER-DEFINED START TIME =================
// Set your desired start time here (HH, MM, SS)
const uint8_t START_HOUR   = 0;
const uint8_t START_MINUTE = 0;
const uint8_t START_SECOND = 0;

// Converted to milliseconds
const unsigned long START_TIME_OFFSET_MS =
  ((unsigned long)START_HOUR   * 3600UL +
   (unsigned long)START_MINUTE * 60UL +
   (unsigned long)START_SECOND) * 1000UL;

// ---- Serial debug helpers ----
#define DBG_BEGIN(baud) do { if (serialDebug) Serial.begin(baud); } while (0)
#define DBG_PRINT(x)    do { if (serialDebug) Serial.print(x); } while (0)
#define DBG_PRINTLN(x)  do { if (serialDebug) Serial.println(x); } while (0)
#define DBG_WRITE(x)    do { if (serialDebug) Serial.write(x); } while (0)

/* ###################     1. LED DRIVER SETUP INITIALIZATION     ################### */

// TLC5947 is a 24-channel 12-bit PWM LED driver.
// NUM_TLC5947 = number of chained boards (each adds 24 channels).
#define NUM_TLC5947 1                   // NUMBER OF LED DRIVERS CHAINED
#define DATA_PIN A3                      // DATA PIN FOR LED DRIVER
#define CLOCK_PIN D2                     // CLOCK PIN FOR LED DRIVER
#define LATCH_PIN D6                     // LATCH PIN FOR LED DRIVER
#define BLANK_PIN D7                     // ANTI BLUR PIN FOR LED DRIVER


// Create TLC5947 instance. Constructor order is (numBoards, clk, data, latch).
Adafruit_TLC5947 tlc = Adafruit_TLC5947(NUM_TLC5947, CLOCK_PIN, DATA_PIN, LATCH_PIN);

/* ###################     2. LUMOS DEVICE INITIALIZATION         ################### */

// Photodiode channels (AS7341) used:
// - 10 visible-ish channels (F1..F8 + NIR + CLEAR) + 1 external analog IR PD => totalPDNum = 11
const int visPDNum   = 10;
const int IRPDNum    = 1;
const int totalPDNum = 11;

// Number of LEDs actively used (<= 24 for a single TLC5947 board).
const int ledNum = 17;

// AS7341 integration settings.
// ATIME/ASTEP determine integration time; see AS7341 datasheet / Adafruit docs.
const int atime = 50;
const int astep = 499;

//INTEGRATION TIMING as added by TH
//integration time as calculated from set parameters atime and astep from AS7341 datasheet
int integration_time = (atime+1)*(astep+1)*(2.78e-3);
//desired reduction in integration time to start checking for state change
int integration_buffer = 2;
//sleep time while waiting for integration
int integration_sleep = integration_time - integration_buffer;
//GET RID OF LATER
int loops = 0;
/* ###################     2.1 DELAYS                             ################### */

// Delay after turning an LED on before reading sensors.
// Should be long enough for LED + sensor integration to be valid.
const int ledOnDelay = 20;

// Delay at end of setup to give hardware time to stabilize.
const int startDelay = 3000;

/* ###################     2.2 LIGHTS UP SEQUENCE                 ################### */

// Grouping / ordering of LED activation.
// Here, groups correspond 1:1 with LEDs (no multi-LED groups).
const int numGroups = 17;
int ledLoopCounter = 0;

// Activation order for LEDs. If you want a custom sweep pattern, reorder this list.
const int ledOrder[numGroups] = {
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
};

int ledBrightness[ledNum] = {
  4095, 4095, 4095, 4095, 4095, 4095,
  1000, 1500, 1500, 1500, 1500, 1500,
  1500, 1500, 1500, 1500, 1500
};

/* ###################     3. VISIBLE PD INITIALIZATION & READINGS PLACEHOLDER      ################### */

// AS7341 spectral sensor object and settings.
// subclass sensor for access to _readingState
AS7341_subclass as7341;
as7341_gain_t gain = AS7341_GAIN_256X;

// States for AS7341 state machine
as7341_waiting_t as7341_currentState;
as7341_waiting_t as7341_previousState;

// Delay used when AS7341 is missing at boot (tight loop).
const int findAS7341Delay = 1000;

uint16_t readings[totalPDNum] = {0};

/* ###################     4. IR PD INITIALIZATION             ################### */

// External IR photodiode connected to analog input.
const int irPDPin = A0;

/* ###################     5. STORAGE CHIP CONFIGURATION             ################### */

// NAND block range to use for logging.
const uint16_t BLOCK_START = 0;
const uint16_t BLOCK_END   = NAND_BLOCK_COUNT - 1;

// Delay used in error-halting loops.
const int nandInitDelay = 1000;

/* ###################     FORWARD DECLS (optional but clean)     ################### */

// NAND routines
void readback();
void erase();

// LED helper
void allLedsOff();

// Data formatting
String combineData(uint16_t readings[], int led, int pwm);

// USB detection (nRF-specific)
static inline bool usbCablePluggedIn();

/*
  allLedsOff():
  - Ensures the TLC5947 is initialized and drives all used LED channels to 0 PWM.
  - This is called during setup and whenever USB is detected.
*/
void allLedsOff() {
  tlc.begin();
  for (int i = 0; i < ledNum; i++) {
    tlc.setPWM(i, 0);
  }
  tlc.write();
}

void setup() {
  DBG_BEGIN(115200);

  // Force LED output to a known safe state at boot.
  allLedsOff();

  // Optional: erase NAND then halt.
  if (eraseBool) {
    erase();
    while (1) { delay(nandInitDelay); } // stop after erase
  }

  // Optional: read back NAND log then halt.
  if (readBackBool) {
    readback();
    while (1) { delay(nandInitDelay); } // stop after readback
  }

  /*
    Normal operation path:
    - Initialize NAND driver
    - Read NAND ID + status
    - Initialize log region
    - Initialize AS7341
    - Initialize TLC5947
  */

  if (!begin()) {
    while (1) {
      delay(nandInitDelay);
      DBG_PRINTLN("NAND begin() failed");
    }
  }

  uint8_t mfg = 0, dev = 0;
  if (!read_ID(mfg, dev)) {
    while (1) {
      delay(nandInitDelay);
      DBG_PRINTLN("NAND read_ID() failed");
    }
  }

  // Read and print NAND status register (implementation is in nand_driver/nand_log libs).
  uint8_t sr = get_status();
  print_status(sr);

  // Initialize logging in the block range.
  if (!log_begin(BLOCK_START, BLOCK_END, false)) {
    while (1) {
      delay(nandInitDelay);
      DBG_PRINTLN("NAND log_begin() failed");
    }
  }

  // Initialize AS7341 spectral sensor.
  if (!as7341.begin()) {
    DBG_PRINTLN("Could not find AS7341");
    while (1) { delay(findAS7341Delay); }
  }

  // Configure AS7341 integration and gain.
  as7341.setATIME(atime);
  as7341.setASTEP(astep);
  as7341.setGain(gain);

  tlc.begin();

  delay(startDelay);
}

void loop() {
    for (int g = 0; g < numGroups; g++) {

    // Select the LED index for this iteration based on ledOrder.
    int led = ledOrder[g];


    /*
      Measurement pass for this LED:
      - Turn LED on at final PWM
      - Log record
      - Turn LED off
    */
    tlc.setPWM(led, ledBrightness[led]);
    tlc.write();
    delay(ledOnDelay);

    // AS7341 integration state machine for high and low channels
    as7341_previousState = as7341.getReadingState(); 

    // sets _readingState to AS7341_WAITING_START
    if (!as7341.startReading()){
      DBG_PRINTLN("Error start reading!");
      return;
    } 

    as7341_currentState = as7341.getReadingState(); 
    
    // if states have changed, sleep for integration, if not, poll
    while(as7341_currentState != AS7341_WAITING_DONE){
      if (as7341_previousState != as7341_currentState){
        delay(integration_sleep);
      } else {
        delay(1);
      }

      as7341_previousState = as7341_currentState;
      as7341.checkReadingProgress();
      as7341_currentState = as7341.getReadingState();
    }
    
    
    //call individual get channel instead of get all channels
    readings[0]  = (int)as7341.getChannel(AS7341_CHANNEL_415nm_F1);
    readings[1]  = (int)as7341.getChannel(AS7341_CHANNEL_445nm_F2);
    readings[2]  = (int)as7341.getChannel(AS7341_CHANNEL_480nm_F3);
    readings[3]  = (int)as7341.getChannel(AS7341_CHANNEL_515nm_F4);
    readings[4]  = (int)as7341.getChannel(AS7341_CHANNEL_555nm_F5);
    readings[5]  = (int)as7341.getChannel(AS7341_CHANNEL_590nm_F6);
    readings[6]  = (int)as7341.getChannel(AS7341_CHANNEL_630nm_F7);
    readings[7]  = (int)as7341.getChannel(AS7341_CHANNEL_680nm_F8);
    readings[8]  = (int)as7341.getChannel(AS7341_CHANNEL_NIR);
    readings[9]  = (int)as7341.getChannel(AS7341_CHANNEL_CLEAR);
    readings[10] = (uint16_t)analogRead(irPDPin);

    String pdData = combineData(readings, led, ledBrightness[led]);

    size_t length = pdData.length();
    bool loggedBool = log_append(
      reinterpret_cast<const uint8_t*>(pdData.c_str()),
      static_cast<uint16_t>(length)
    );

    if (!loggedBool) {
      DBG_PRINTLN("NAND log_append() failed!");
    }

    DBG_PRINTLN(pdData);

    // Turn LED off after logging.
    tlc.setPWM(led, 0);
    tlc.write();
    delay(ledOnDelay);
  }

}

/*
  combineData():
  - Builds a single log record string with:
      timestamp, all PD readings, [ledIndex], pwm;
  - Timestamp is time since boot in HH-MM-SS.mmm format.
  - Trailing ';' is used by readback() to split records into lines.
*/
String combineData(uint16_t readings[], int led, int pwm) {
  String pdData = "";

  // Time since boot (HH-MM-SS.mmm)
  unsigned long ms = millis() + START_TIME_OFFSET_MS;
  unsigned long totalSeconds = ms / 1000UL;
  unsigned int hours = (totalSeconds / 3600UL) % 24;
  unsigned int minutes = (totalSeconds % 3600UL) / 60UL;
  unsigned int seconds = totalSeconds % 60UL;
  unsigned int msec    = ms % 1000UL;

  char timeBuf[16];
  snprintf(timeBuf, sizeof(timeBuf), "%02u-%02u-%02u.%03u",
          hours, minutes, seconds, msec);

  // Prefix timestamp
  pdData += String(timeBuf) + ",";

  // Append all PD channels
  for (int i = 0; i < totalPDNum; i++) {
    pdData += String(readings[i]) + ",";
  }

  // Append LED index in brackets (easy to parse)
  pdData += "[";
  pdData += String(led);
  pdData += "],";

  // Append PWM
  pdData += String(pwm);

  // Record terminator (used by readback() to print one record per line)
  pdData += ";";

  return pdData;
}

/*
  readback():
  - Opens NAND log for reading and iterates through stored records.
  - Prints each record and attempts to format output so each ';' ends a line.
  - Uses log_iter_next() from nand_log library to step through records.
*/
void readback() {
  // Wait briefly for Serial on boards that enumerate USB serial.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 3000)) { delay(10); }

  DBG_PRINTLN();
  DBG_PRINTLN("=== READBACK START ===");

  if (!begin()) {
    DBG_PRINTLN("ERROR: NAND begin() failed");
    return;
  }

  if (!log_begin(BLOCK_START, BLOCK_END, false)) {
    DBG_PRINTLN("ERROR: log_begin() failed");
    return;
  }

  // Reset the iterator to the beginning of the log region.
  log_iter_reset();

  // Max record size derived from NAND page size minus log header.
  // This assumes records are stored within a NAND “main area” page.
  const uint16_t maxRecordSize = NAND_MAIN_BYTES - sizeof(LogHdr);

  // Allocate a buffer for reading each record.
  uint8_t* buf2 = new uint8_t[maxRecordSize];
  uint16_t n = 0;
  int recordCount = 0;

  // Iterate through all records; n is the number of bytes returned into buf2.
  while (log_iter_next(buf2, maxRecordSize, &n)) {
    recordCount++;

    DBG_PRINT("Record ");
    DBG_PRINT(recordCount);
    DBG_PRINT(" (n=");
    DBG_PRINT(n);
    DBG_PRINTLN("):");

    // Print buffer content; split by ';' so each logical record ends on its own line.
    for (uint16_t i = 0; i < n; ++i) {
      char c = (char)buf2[i];

      // Skip embedded NULs if any.
      if (c == '\0') continue;

      if (c == ';') {
        Serial.write(';');
        Serial.write('\n');
      } else {
        Serial.write(c);
      }
    }

    Serial.write('\n');  // end of record
    DBG_PRINTLN("---");
    delay(1); // Small delay can improve reliability on some USB serial stacks.
  }

  delete[] buf2;

  DBG_PRINT("Total records read: ");
  DBG_PRINTLN(recordCount);
  DBG_PRINTLN("=== READBACK END ===");
}

/*
  erase():
  - Erases every NAND block (0..NAND_BLOCK_COUNT-1).
  - This is destructive and should typically be used only when requested.
*/
void erase() {
  begin();
  for (uint16_t b = 0; b < NAND_BLOCK_COUNT; ++b) {
    erase_block(b);
  }
  DBG_PRINTLN("All blocks erased.");
}