/**
 * \file M5wire.cpp
 *
 */
#include "M5wire.h"

/**
 * Begin to use device.
 * Can't be called in constructor because Wire library is
 * not functional at this point.
 */
void M5w_Unit::begin() 
{
	wire.begin();
	delay(10);
	wire.beginTransmission(addr);
	error = wire.endTransmission();    
}

/**
 * LED write stalls I²C transactions for a while - delay
 * if we're too close to the last time we did one.
 * An LED write is 5 or 6 bytes long, so with a 100kHz
 * I²C bus it takes ~500us for the command, plus 
 * (empirically) 80us for the delay if it's needed.
 */
void M5w_Unit::addDelayIfNeeded(void)
{
	uint32_t now = micros();

	if (now - lastLEDtime < LEDdelay)
		delayMicroseconds(LEDdelay - (now - lastLEDtime));
}
	

/**
 * Read a value from a register
 * \return 0 if read OK, other value if it failed
 */
uint8_t M5w_Unit::readBytes(uint8_t reg,	//!< register address within the unit
							uint8_t* value, //!< pointer to storeage for value to read
							uint8_t n)		//!< number of bytes in register
{
	addDelayIfNeeded();

	wire.beginTransmission(addr);
	wire.send(reg);
	endError = wire.endTransmission();

	if (0 == endError)
	{
	if (wire.requestFrom(addr, (uint8_t) n))
	{
	  for (int i=0;i<n;i++)
		value[i] = Wire.read();
	}
	else
	  endError=10;
	}
	else
	endError+=100;

	return endError;
}


/**
 * Read single-byte register
 * \return register value
 */
uint8_t M5w_Unit::getByte(uint8_t reg) //!< register address within the unit
{
	uint8_t ret;
	if (0 != readBytes(reg,&ret,1))
		ret = 0xAA;
	return ret;
}


/**
 * Read 4-byte register
 * \return register value
 */
int32_t M5w_Unit::getLong(uint8_t reg) //!< register address within the unit
{
	int32_t ret;
	if (0 != readBytes(reg,(uint8_t*) &ret,4))
		ret = 0xDEADBEEF;
	return ret;
}


/**
 * Write a value to a register
 */
void M5w_Unit::writeBytes(uint8_t reg,		//!< register address within the unit
						  uint8_t* value, 	//!< pointer to value to write
						  uint8_t n)		//!< number of bytes in register
{
	addDelayIfNeeded();

	wire.beginTransmission(addr);
	wire.write(reg);
	wire.write(value,n);
	endError = wire.endTransmission();    
}

//====================================================
/**
 * Get value of one of the 8 potentiometers
 * NOTE: this reverses the sense of the register value, as
 * otherwise it runs from about 4095 at the left limit,
 * down to 0 at the right. Which is weird...
 * \return value from 0..4095
 */
uint16_t M5w_8angle::getPot16(uint8_t ch) //!< potentiometer number, 0..7
{
  uint16_t value = potValues[ch];
  
  if (0 == readBytes(0x0 + ch*2,(uint8_t*) &value,2))
  {
    value = value > POTMAX?0:POTMAX-value;
	switch (hooks[ch])
	{
	  case unhooked:
		potValues[ch] = value;
		break;
		
	  case hooking:
	    if (value < potValues[ch])
			hooks[ch] = under;
		else if (value > potValues[ch])
			hooks[ch] = over;
		else
		{
			hooks[ch] = unhooked;
			potValues[ch] = value;
		}
		break;
		
	  case under:
		if (value >= potValues[ch])
		{
			hooks[ch] = unhooked;
			potValues[ch] = value;
		}
		break;
		
	  case over:
		if (value <= potValues[ch])
		{
			hooks[ch] = unhooked;
			potValues[ch] = value;
		}
		break;
	}
  }

  return potValues[ch];
}

/**
 * Set pot hook point.
 * Set value and state such that the value is always returned by
 * ::getPot16() until the pot is turned through the hook point, 
 * after which it is unhooked and the returned value will
 * reflect the physical position.
 */
void M5w_8angle::setHook(uint8_t ch, uint16_t value)
{
	hooks[ch] = hooking;
	potValues[ch] = value;
}
 


/**
 * Get switch state
 * \return 0 or 1
 */
uint8_t M5w_8angle::getSwitch(void)
{
  return getByte(0x20);
}


/**
 * Set LED colour
 */
void M5w_8angle::writeLED(uint8_t led,		//!< LED number, 0..8
						  uint32_t colour)	//!< colour: <brightness><B><G><R>
{
	writeBytes(led*4+0x30,(uint8_t*) &colour,4);
	lastLEDtime = micros();
}


//====================================================
/**
 * Get counter value
 * \return counter value,range ±2Gi
 */
int32_t M5w_8encoder::getCount(uint8_t ch) //!< counter channel
{
	return getLong(0x00 + 4*ch);
}


/**
 * Reset counter value
 */
void M5w_8encoder::resetCount(int8_t ch) //!< counter channel
{
	resetCounts(1<<ch);
}


/**
 * Reset multiple counter values
 */
void M5w_8encoder::resetCounts(uint8_t mask) //!< counter channels as bitmask
{
	uint8_t chList[8];
	uint8_t reg = 0x40;
	uint8_t len = 0;
	
	if (0 != mask)
	{
		// find start register
		while (0 == (mask & 1))
		{
			mask >>= 1;
			reg++;
		}
		
		// set reset flags for chosen channels
		while (0 != mask)
		{
			chList[len] = (mask & 1) ?1:0;
			len++;
			mask >>= 1;
		}
		
		// send command to device
		writeBytes(reg,chList,len);
	}	
}


/**
 * Get increment value
 * Increment is reset to 0 after the read
 * \return increment value,range ±2Gi
 */
int32_t M5w_8encoder::getIncrement(uint8_t ch) //!< increment channel
{
	return getLong(0x20 + 4*ch);
}


/**
 * Get encoder button state(s)
 * Buttons read inverted, we flip them so 1 means pressed.
 * \return 0 or 1, or bitmap if reading channel -1
 */
uint8_t M5w_8encoder::getButton(int8_t ch) //!< encoder channel, or -1 to get all
{
	uint8_t ret = 0;
	if (ch >= 0)
		ret = getByte(0x50 + ch) == 0;
	else
		for (int i=7;i>=0;i--)
			ret = (ret << 1) | ((getByte(0x50 + ch) == 0) ?1:0);
			
	return ret; 
}


/**
 * Set LED colour
 */
void M5w_8encoder::writeLED(uint8_t led,	//!< LED number, 0..8
						  uint32_t colour)	//!< colour: <B><G><R>
{
	writeBytes(led*3+0x70,(uint8_t*) &colour,3);
	lastLEDtime = micros();
}


/**
 * Get switch state
 * \return 0 or 1
 */
uint8_t M5w_8encoder::getSwitch(void)
{
  return getByte(0x60);
}

