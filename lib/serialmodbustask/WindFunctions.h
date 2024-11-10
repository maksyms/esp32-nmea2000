#ifndef WindFunctions_h
#define WindFunctions_h
#include <Arduino.h>

// Control RS485 wind direction and speed sensors with arduino.
class WindFunctions
{
public:
	// Variables
	static constexpr char dirList[][5] = {"   N", " NNE", "  NE", " ENE", "   E", " ESE", "  SE", " SSE", "   S", " SSW", "  SW", " WSW", "   W", " WNW", "  NW", " NNW", "   N", "INIT", "FAIL"};

	int16_t WindSpeed;
	int16_t WindScale;
	int16_t WindAngle;
	int16_t WindDirection;

	uint8_t ReadAddr; // TODO: not updated

	// Functions
	WindFunctions(Stream *s, uint8_t addr) : serial(s), address(addr) {};
	bool readAll() { return readAll(address); };
	bool readAll(uint8_t address);

	// TODO: Deprecated, as it is for some other sensor
	// Read Wind Speed
	int16_t readWindSpeed() { return readWindSpeed(address); };
	int16_t readWindSpeed(uint8_t address);
	// Read Wind Scale
	int16_t readWindScale() { return readWindScale(address); };
	int16_t readWindScale(uint8_t address);
	// Read Wind direction as 360 degrees, but you must divide by 10 to get the right number.
	int16_t readWindDirection360() { return readWindDirection360(address); };
	int16_t readWindDirection360(uint8_t address);
	// Read Wind Direction as 16 directions
	int16_t readWindDirection16() { return readWindDirection16(address); };
	int16_t readWindDirection16(uint8_t address);
	// Read the address of the currently connected device.
	uint8_t readAddress();
	// modifies the address of the connected unit. Only one at a time. Restart when done.
	boolean ModifyAddress(uint8_t Address1, uint8_t Address2);

private:
	// Functions
	size_t readN(uint8_t *buf, size_t len);
	uint16_t CRC16_2(uint8_t *buf, int16_t len);
	void addedCRC(uint8_t *buf, int len);
	Stream *serial;
	uint8_t address;
};

#endif
