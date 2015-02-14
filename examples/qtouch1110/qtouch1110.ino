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


Do you know how to set up the chip?

Basically, hook up power and gnd, and put a .1uF cap across them (near the chip).  
Then hook up the SPI:
MISO arduino pin 12 -> QT1110 pin 16, 
MOSI arduino pin 11 -> QT1110 pin 15,
SCK arduino pin 13  -> QT1110 pin 17, 
SS (your choice, often arduino pin 10, or mega pin 53 -> QT1110 pin 14).    

Now, you need to hook up a touch plate.  I just soldered a wire to an unetched PCB board (I used one that was about 2cm square).  Put that into 4.7k resistor.  From there put a 20nF (or so) cap to pin 2.  And put a wire from the resistor to pin 3.  Check the QT1110 datasheet for the other cap sensing wire pairs.
*/

#include <qtouchArduino.h>
#include <SPI.h>

#define DbgPrt p

#define MAX_STR_SIZE 80

void p(const char *fmt, ... )
{        
  char tmp[MAX_STR_SIZE+1];
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, MAX_STR_SIZE, fmt, args);
  va_end (args);
  tmp[MAX_STR_SIZE] = 0;
  //Usb.print(tmp);
  Serial.print(tmp);
}


void setup(void)
{
  Serial.begin(9600);
}


#define SPI_CHIP_SELECT_PIN 10

void loop(void)
{
  //Qt1110 qt(SPI_CHIP_SELECT_PIN,11,0);  // 11 keys, no guard key

#if 1  // Alternate (verbose) initialization mechanism
  Qt1110 qt(SPI_CHIP_SELECT_PIN);
  qt.reset();
  byte rev = qt.validate();
  DbgPrt("Chip Firmware rev: %x (%d.%d)\n",rev,rev>>4,rev&0xf);

  qt.configure(11,0);
  qt.calibrate();
#endif


  uint16_t old=0;
  
  for (uint16_t cnt;;cnt++) //int i=0;i<100;i++)
    {
      // Lets get a bitmap of every key's state
      uint16_t keymap = qt.getAllKeys();

      // And get the key number (if multiple keys are pressed this only gets you the first)
      uint8_t key = qt.getKey();
      if (keymap != old)  // Something changed
	{
          DbgPrt("Key bitmap: 0x%x first key touched: %d\n", keymap,key);
          old=keymap;
	}
      delay(1); 

      if ((cnt&0xff)==0)
        {
        uint16_t prox0 = qt.getProximity(1);
        uint16_t prox1 = qt.getProximity(2);
        DbgPrt("key 1: %d  Key 2: %d\n", prox0, prox1);
        }
        
      if ((cnt&0x3ff)==0)
        {        
        DbgPrt("key: status:");
        for (int i=0;i<11;i++)
          {
          uint8_t status = qt.getStatus(i);
          DbgPrt("%2d: 0x%2x ", i, status);
          }
        DbgPrt("\n");
        }
      // Periodically print chip's status
      if ((cnt&0x7ff)==0)
	{
          uint16_t ret;
          DbgPrt("Status: ");
	  QTouch::DeviceStatus status = qt.getStatus();
          if (status.detect) DbgPrt("Some key is touched; ");
          if (status.cycle) DbgPrt("cycle time overrun; ");
          if (status.error) DbgPrt("some key is errored; ");     // Expected unless you have hooked up EVERY key
          if (status.change) DbgPrt("change pin; ");
          if (status.eeprom) DbgPrt("EEPROM error; ");           // Expected since we aren't using it (yet).
          if (status.eeprom) DbgPrt("device just reset; ");
          if (status.guard) DbgPrt("guard is detecting; ");
          DbgPrt("\n");
          ret = qt.getAllErrors();
	    DbgPrt("Errored key bitmap (unused keys are seen as errored): 0x%x\n",ret);
	}
    }
}
