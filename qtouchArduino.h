/*
    Copyright (c) 2012, Toasted Circuits, Inc.

    This file is part of the qtouchArduino Library.

    The qtouchArduino Library is free software: you can redistribute it and/or 
    modify it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, version 3 of the License.

    The qtouchArduino Library is distributed in the hope that it will be 
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with this software.  If not, see <http://www.gnu.org/licenses/>.

    Author: G. Andrew Stone
*/

#ifndef QTOUCH1110_H
#define QTOUCH1110_H

#include "Arduino.h"
#include <ctype.h>

namespace QTouch
{

  typedef struct
  {
    uint8_t guard:1;
    uint8_t reset:1;
    uint8_t eeprom:1;
    uint8_t change:1;

    uint8_t error:1;
    uint8_t cycle:1;
    uint8_t detect:1;
    uint8_t one:1;
  } DeviceStatus;
}

class Qt1110
{
  uint8_t chipSelect;
  void handleInitError(void);

public:
  Qt1110();  // For initialization of arrays; use init() to construct
  Qt1110(uint8_t chipSelectPin);
  Qt1110(uint8_t chipSelectPin,uint8_t numkeys, uint8_t guardkey);  // Construct, configure & calibrate

  // These init function do the exact same thing as the constructor, and are used when arrays are created.
  void    init(uint8_t chipSelectPin);
  void    init(uint8_t chipSelectPin,uint8_t numkeys, uint8_t guardkey); 
  
  void    configure(uint8_t numkeys, uint8_t guardkey); 
  
  void    pause();  // Temporarily stop using SPI for this device
  void    resume(); // Continue using SPI
  void    reset(void);  
  uint8_t test(void);

  boolean validate(void);

  // NOTE: all KEY numbers are 1 based (not zero based like in the QT1110 spec)

  // Return a bitmap 1=detected, 0=not detected.  So to see if key 3 is "pressed": if (getKeyState() & (1<<3)) { print("key 3 pressed"); }
  uint16_t getAllKeys(void);
  // This will return a key number in detect.  But if multi-press it only returns the FIRST key pressed.  Use getAllKeys() for multidetect
  uint8_t getKey(void);
  //uint8_t getKey(uint8_t key);

  // Return a bitmap 1=detected, 0=not detected.  So to see if key 3 is "pressed": if (getKeyState() & (1<<3)) { print("key 3 pressed"); }
  uint16_t getAllErrors(void);

  // (optional) Call this when nobody is touching the keys :-).  If you change the chip's mode (# of keys), you must either recalibrate or load new calibration into the chip
  void calibrate();
  void calibrate(uint8_t key);

  void save(void);  // Save your settings into the chip's EEPROM
  void load(void);  // Load setting from EEPROM: not really necessary because your settings are loaded when the chip comes out of reset
  
  

  // Get the chip's status bits.  Look at the chip spec, or the example code for a definition of these bits.
  QTouch::DeviceStatus getStatus(void);

  // Not really part of the "public" API
  void writeSetup(uint8_t addr, uint8_t data);
};

#endif
