#pragma once
#include <Adafruit_AS7341.h>

// this subclass makes the reading state of the AS7341 public 
class AS7341_subclass : public Adafruit_AS7341 {
  public: 
  as7341_waiting_t getReadingState() const {return _readingState; }
};