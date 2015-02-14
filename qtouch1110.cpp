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

#include "qtouchArduino.h"
#include <ctype.h>
#include "SPI.h"

#define DbgPrt(...) p 

extern void p(const char *fmt,...);

namespace QTouch
{
  enum QT1110Misc
  {
    OK = 0x55,
    DeviceId = 0x57,
    DeviceCommDelay = 150
  };

  enum QT1110Commands
  {
    // Control
    SendSetups = 0x1,
    CalibrateAll = 0x3,
    ChipReset = 0x4,
    Sleep = 0x5,
    SaveToEeprom = 0xA,
    LoadFromEeprom = 0xB,
    EraseEeprom    = 0xc,
    RecoverEeprom  = 0xd,
    CalibrateKeyN   = 0x10,
  
    // Report
    SendFirstKey = 0xC0,
    SendAllKeys  = 0xC1,
    DeviceStatusCmd = 0xC2,
    EepromCrc    = 0xC3,
    RamCrc       = 0xC4,
    ErrorKeys    = 0xC5,
    SignalKeyN   = 0x20,
    RefKeyN      = 0x40,
    StatusKeyN   = 0x80,
    WriteCmd     = 0x90,
    DetectOutputStates = 0xC6,
    LastCommand  = 0xC7,
    GetSetups    = 0xC8,
    GetDeviceId     = 0xC9,
    GetFirmwareVer  = 0xCA
  };

  enum QT1110HardCodedSetup
  {
    KEY_AC      = 1,
    SIGNAL      = 1,
    SYNC        = 1,
    REPEAT_TIME = 8,

    QUICK_SPI   = 0,
    CHG         = 1,
    CRC         = 0
  };
  
};

#define DevicePrecommDelay()  // delayMicroseconds(DeviceCommPreDelay);
#define CalibrateKey(x) (QTouch::CalibrateKeyN + x)



Qt1110::Qt1110(uint8_t chipSelectPin)
{
  init(chipSelectPin);
}

void Qt1110::init(uint8_t chipSelectPin)
{
  chipSelect = chipSelectPin;
  pinMode(chipSelect,OUTPUT);
  
  resume();
}

Qt1110::Qt1110()
{
  chipSelect=0xff;
}

Qt1110::Qt1110(uint8_t chipSelectPin,uint8_t numkeys, uint8_t guardkey)
{
  init(chipSelectPin,numkeys,guardkey);
}

void Qt1110::init(uint8_t chipSelectPin,uint8_t numkeys, uint8_t guardkey)
{
  chipSelect = chipSelectPin;
  pinMode(chipSelect,OUTPUT);
  
  resume();

  reset();
  byte rev = validate();
  configure(numkeys, guardkey);
  calibrate();
}


void Qt1110::configure(uint8_t numkeys, uint8_t guardkey)
{
  if (numkeys==11) numkeys=1;
  else numkeys=0;
  
  uint8_t data =  (QTouch::KEY_AC<<7) | (numkeys<<6) | (QTouch::SIGNAL<<5) | (QTouch::SYNC<<4) | (QTouch::REPEAT_TIME&0xf);
  writeSetup(0, data);

  if (guardkey>0) data =  (((guardkey-1)&0xf)<<4) | (1<<3);  // set the guard key number and GD_EN
  else data = 0; 
  data |= (QTouch::QUICK_SPI<<2) | (QTouch::CHG<<1) | (QTouch::CRC<<0);
  writeSetup(1, data);

  // Atmel firmware bug requires a reset (or maybe a load()) to actually switch the mode
  save();
  reset();

}

void Qt1110::writeSetup(uint8_t addr, uint8_t data)
{
  if (addr>41) 
    {
      DbgPrt("Out of range EEPROM address: %d",addr);
      return;
    }

  DevicePrecommDelay();
  uint8_t ret = 0;
  //while (ret!=0x55) 
    {
    ret = SPI.transfer(QTouch::WriteCmd + addr);
    //if (ret != 0x55) delayMicroseconds(QTouch::DeviceCommDelay);
    }
  
  delayMicroseconds(QTouch::DeviceCommDelay);
  SPI.transfer(data);
}

uint16_t Qt1110::getProximity(int key)
{
  DevicePrecommDelay();
  SPI.transfer(QTouch::SignalKeyN + key);
  delayMicroseconds(QTouch::DeviceCommDelay);
  uint16_t msb = SPI.transfer(0);
  delayMicroseconds(QTouch::DeviceCommDelay);
  uint16_t lsb = SPI.transfer(0);
  DevicePrecommDelay();
  SPI.transfer(QTouch::LastCommand);
  //uint16_t cmd = SPI.transfer(0);
  //return cmd;
  return (msb<<8 | lsb);
}

QTouch::DeviceStatus Qt1110::getStatus(void)
{
  union
  {
    uint8_t c;
    QTouch::DeviceStatus stat;
  } ret;

  DevicePrecommDelay();
  SPI.transfer(QTouch::DeviceStatusCmd);
  delayMicroseconds(QTouch::DeviceCommDelay);
  ret.c = SPI.transfer(0);
  return ret.stat;
}

void Qt1110::save(void)
{
  DevicePrecommDelay();
  SPI.transfer(QTouch::SaveToEeprom);
  delay(50);  
}
void Qt1110::load(void)
{
  DevicePrecommDelay();
  SPI.transfer(QTouch::LoadFromEeprom);
  delay(150);  
}

uint8_t Qt1110::getStatus(uint8_t key)
{  
  DevicePrecommDelay();
  SPI.transfer(QTouch::StatusKeyN + key);
  delayMicroseconds(QTouch::DeviceCommDelay);
  uint8_t ret = SPI.transfer(0);
  return ret;
}

uint8_t Qt1110::getKey(void)
{  
  DevicePrecommDelay();
  SPI.transfer(QTouch::SendFirstKey);
  delayMicroseconds(QTouch::DeviceCommDelay);
  uint8_t ret = SPI.transfer(0);
  if ((ret&0x80)==0) return 0;
  return (ret&0xf)+1;
}

void Qt1110::calibrate(void)
{
  DevicePrecommDelay();
  SPI.transfer(QTouch::CalibrateAll);
  delayMicroseconds(QTouch::DeviceCommDelay);
}

void Qt1110::calibrate(uint8_t key)
{
  if (key>=11) 
    {
      DbgPrt("Calibrate: key %d does not exist\n",key);
      return;
    }

  DevicePrecommDelay();
  SPI.transfer(CalibrateKey(key));
  delayMicroseconds(QTouch::DeviceCommDelay);
}


uint16_t Qt1110::getAllKeys(void)
{
  uint16_t ret;
  delayMicroseconds(QTouch::DeviceCommDelay); // In multibyte communications, the master must pause for a minimum delay of 150 us between the completion of one byte exchange and the beginning of the next.
  ret = SPI.transfer(QTouch::SendAllKeys);
  delayMicroseconds(QTouch::DeviceCommDelay);
  ret = SPI.transfer(0)<<8;
  delayMicroseconds(QTouch::DeviceCommDelay);
  ret |= SPI.transfer(0);
  //DbgPrt("QTouch key state: %x\n",ret);
  return (ret<<1);  // Shift the bitmap by one bit to 
}

uint16_t Qt1110::getAllErrors(void)
{
  uint16_t ret;
  delayMicroseconds(QTouch::DeviceCommDelay); // In multibyte communications, the master must pause for a minimum delay of 150 us between the completion of one byte exchange and the beginning of the next.
  ret = SPI.transfer(QTouch::ErrorKeys);
  delayMicroseconds(QTouch::DeviceCommDelay);
  ret = ((uint16_t)SPI.transfer(0))<<8;
  delayMicroseconds(QTouch::DeviceCommDelay);
  ret |= SPI.transfer(0);
  //DbgPrt("QTouch key state: %x\n",ret);
  return (ret<<1);  // Shift the bitmap by one bit to 
}



void Qt1110::reset(void)
{
  SPI.transfer(QTouch::ChipReset);
  delay(QTouch::DeviceCommDelay);
}

uint8_t Qt1110::test(void)
{
  uint8_t ret;
  uint8_t cnt;
  DbgPrt("QTTEST\n");

  for (byte order = 0; order < 2; order ++)
    {
      if (order==0) SPI.setBitOrder(MSBFIRST);  // With these 8 clock pulses, a byte of data is transmitted from the master to the slave over MOSI, most significant bit (msb) first.
      else SPI.setBitOrder(LSBFIRST);
      for (byte datamode=0;datamode<4;datamode++)
	{
	  SPI.setDataMode(datamode);  //3);  // CPOL | CPHA
	  for (byte clkDivider=SPI_CLOCK_DIV4; clkDivider <= SPI_CLOCK_DIV32; clkDivider++)
	    {
	      SPI.setClockDivider(SPI_CLOCK_DIV32); // 16mhz/16 = 1mhz: The QT1110 SPI interface can operate at any SCK frequency up to 1.5 MHz.
  
	      digitalWrite(chipSelect,0);
	      SPI.begin();

	      DbgPrt("Bit Order %d, datamode: %d, clkdiv: %d\n ",order,datamode,clkDivider);
    
	      for (cnt = 0; cnt < 100; cnt++)
		{
		  delayMicroseconds(QTouch::DeviceCommDelay); // In multibyte communications, the master must pause for a minimum delay of 150 us between the completion of one byte exchange and the beginning of the next.
		  ret = SPI.transfer(cnt);
		  DbgPrt(" %x",ret);
		}
	    }
	}
    }
}

void Qt1110::handleInitError(void)
{
  // What if the SPI on the slave has a spurious bit clocked in

  DbgPrt("init error\n");
  
  for (byte i=0;i<100;i++)
    {
      SPI.end();
      //digitalWrite(SCK,1);
      //delayMicroseconds(5);
      digitalWrite(SCK,0);
      delayMicroseconds(5);
      digitalWrite(SCK,1);
      delayMicroseconds(5);
      resume();
      SPI.transfer(0);
      if (SPI.transfer(0) == QTouch::OK) break;
    }
  
}

uint8_t Qt1110::validate(void)
{
  uint8_t ret;
  uint8_t cnt;
  // Validate the device identifier
  for (cnt = 0; cnt < 30; cnt++)
    {
      delayMicroseconds(QTouch::DeviceCommDelay); // In multibyte communications, the master must pause for a minimum delay of 150 us between the completion of one byte exchange and the beginning of the next.
      ret = SPI.transfer(QTouch::GetDeviceId);
      //DbgPrt("ret: %d\n",ret);
      if (ret != QTouch::OK) { handleInitError(); continue; }
      delayMicroseconds(QTouch::DeviceCommDelay);
      ret = SPI.transfer(0);
      DbgPrt("QTouch device Id: %x\n",ret);
      if (ret == QTouch::DeviceId) break;    
    }
  if (cnt>9) return 0;

  for (cnt = 0; cnt < 10; cnt++)
    {
      delayMicroseconds(QTouch::DeviceCommDelay);
      ret = SPI.transfer(QTouch::GetFirmwareVer);
      //DbgPrt("retr: %x\n",ret);
      if (ret == QTouch::OK)
	{
          delayMicroseconds(QTouch::DeviceCommDelay);
 	  ret = SPI.transfer(0);  // This is the firmware rev
	  break;
	}
    }
  if (cnt>9) return 0;

  return ret;
}


void Qt1110::resume(void)
{
  SPI.setBitOrder(MSBFIRST);  // With these 8 clock pulses, a byte of data is transmitted from the master to the slave over MOSI, most significant bit (msb) first.
  SPI.setDataMode(SPI_MODE3);  //3);  // CPOL | CPHA
  SPI.setClockDivider(SPI_CLOCK_DIV32); // 16mhz/16 = 1mhz: The QT1110 SPI interface can operate at any SCK frequency up to 1.5 MHz.
  SPI.begin();  // writes SS high
  //SPCR |= _BV(MSTR);
  //SPCR |= _BV(SPE);
  digitalWrite(chipSelect,0);
}

void Qt1110::pause(void)
{
  digitalWrite(chipSelect,1);
}

