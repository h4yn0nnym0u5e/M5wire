/**
 * \file M5wire.cpp
 *
 */
#if !defined(_M5WIRE_H_)
#define _M5WIRE_H_
 
#include <Wire.h>

//====================================================
class M5w_Unit {
	uint8_t addr;
	TwoWire& wire;
	uint8_t error;
	uint8_t endError;
	void addDelayIfNeeded(void);

  protected:  
	uint32_t lastLEDtime;
	
  public:
	M5w_Unit(uint8_t ad, TwoWire& w = Wire) 
		: addr(ad),wire(w),error(0), endError(0), LEDdelay(80)
		{}

	uint32_t LEDdelay;

	void begin(); 
	uint8_t readBytes(uint8_t reg,uint8_t* value, uint8_t n);
	void writeBytes(uint8_t reg,uint8_t* value, uint8_t n);
	uint8_t getByte(uint8_t reg);
	int32_t getLong(uint8_t reg);

	// simple inline functions
	explicit operator bool() {return error == 0;}  		// check that unit is functioning
	uint8_t getVersion(void) { return getByte(0xFE); }	// return firmware version
	uint8_t getAddress(void) { return getByte(0xFF); }	// return device address
	void setAddress(uint8_t newAddr) { writeBytes(0xFF,&newAddr,1); addr = newAddr; } // set device address
};


  
//====================================================
class M5w_8angle : public M5w_Unit
{
  static const int POT_COUNT = 8;
  const uint16_t POTMAX = 0xFFC;
  uint16_t potValues[POT_COUNT] = {0};
  enum {unhooked,under,over,hooking} hooks[POT_COUNT];
  
public:  
  M5w_8angle(uint8_t ad = 0x43, TwoWire& w = Wire) : M5w_Unit(ad, w),hooks{unhooked} {}
  
  uint16_t getPot16(uint8_t ch);
  uint16_t getLast(uint8_t ch) { return potValues[ch]; }
  uint8_t getSwitch(void);
  void setHook(uint8_t ch, uint16_t value);
  bool isHooking(uint8_t ch) { return hooks[ch] != unhooked; };
  void clearHook(uint8_t ch) { hooks[ch] = unhooked; };
  void writeLED(uint8_t led,uint32_t colour);
};


  
//====================================================
class M5w_8encoder : public M5w_Unit
{
	int32_t encValues[8] = {0};
  
public:  
	M5w_8encoder(uint8_t ad = 0x41, TwoWire& w = Wire) : M5w_Unit(ad, w) {}

	int32_t getCount(uint8_t ch);
	void setCount(int8_t ch,int32_t count);
	int32_t getIncrement(uint8_t ch);  
	void resetCount(int8_t ch);
	void resetCounts(uint8_t mask);
	uint8_t getSwitch(void);
	uint8_t getButton(int8_t ch);
	uint8_t getButtons(void) { return getButton(-1); }
	void writeLED(uint8_t led,uint32_t colour);
};

#endif // !defined(_M5WIRE_H_)
