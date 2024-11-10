#include <WindFunctions.h>
#include <Arduino.h>

size_t WindFunctions::readN(uint8_t *buf, size_t len)
{
  size_t offset = 0;
  long start_time = millis(); // Start time for timeout check

  // Wait until enough data is available, with a timeout check
  while (serial->available() < len)
  {
    if (millis() - start_time > 100)
    {        // 100ms timeout (adjust as needed)
      break; // Exit if data isn't received within the timeout
    }
  }

  // Read all available bytes (up to len)
  while (offset < len && serial->available())
  {
    buf[offset] = serial->read();
    offset++;
  }

  return offset; // Return the number of bytes successfully read
}

// Calculate CRC16_2 check value
// buf: Packet for calculating the check value
// len: Check the data length
// return: Return a 16-bit check result
uint16_t WindFunctions::CRC16_2(uint8_t *buf, int16_t len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }

  crc = ((crc & 0x00ff) << 8) | ((crc & 0xff00) >> 8);
  return crc;
}

void WindFunctions::addedCRC(uint8_t *buf, int len)
{
  uint16_t crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++)
  {
    crc ^= (uint16_t)buf[pos];
    for (int i = 8; i != 0; i--)
    {
      if ((crc & 0x0001) != 0)
      {
        crc >>= 1;
        crc ^= 0xA001;
      }
      else
      {
        crc >>= 1;
      }
    }
  }
  buf[len] = crc % 0x100;
  buf[len + 1] = crc / 0x100;
}

// Read all the data available from the sensor. 1 parameter is address.
bool WindFunctions::readAll(uint8_t address)
{
  bool bSuccess = false; // Data acquisition success flag

  uint8_t requestData[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x04, 0x44, 0x09}; // Command for reading wind speed - this is the original for XS-WSDS01 sensor
  uint8_t returnData[14] = {0};                                              // Store the original data packet returned by the sensor
  uint8_t ch = 0;

  requestData[0] = address; // Add the complete command package with reference to the communication protocol.
  addedCRC(requestData, 6); // Add CRC_16 check for reading wind speed command packet
  uint8_t retries = 0;

  while (!bSuccess)
  {
    if (retries > 2)
      break;

    serial->write(requestData, sizeof(requestData) / sizeof(requestData[0])); // Send the command of reading the wind speed
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == address) // The first byte is the address of the sensor
      {
        returnData[0] = ch;
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x03) // The second byte is the function code
          {
            returnData[1] = ch;
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x08) // The third byte is data length of all the registers - we have 4 registers, 2 bytes each
              {
                returnData[2] = ch;
                if (readN(&returnData[3], 10) == 10) // Read all the registers
                {
                  if (CRC16_2(returnData, 11) == ((returnData[11] << 8) | returnData[12])) // Check CRC of the returned data
                  {
                    // Store obtained data
                    WindSpeed = ((returnData[3] << 8) | returnData[4]);
                    WindScale = ((returnData[5] << 8) | returnData[6]);
                    WindAngle = ((returnData[7] << 8) | returnData[8]);
                    WindDirection = ((returnData[9] << 8) | returnData[10]);
                    bSuccess = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return bSuccess;
}

// Read the wind speed. 1 parameter is address.
int16_t WindFunctions::readWindSpeed(uint8_t address)
{
  uint8_t Data[7] = {0};                                             // Store the original data packet returned by the sensor
  uint8_t COM[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; // Command for reading wind speed
  boolean ret = false;                                               // Wind speed acquisition success flag
  uint8_t ch = 0;
  COM[0] = address; // Add the complete command package with reference to the communication protocol.
  addedCRC(COM, 6); // Add CRC_16 check for reading wind speed command packet
  int retries = 0;

  while (!ret)
  {
    if (retries > 2)
    {
      WindSpeed = -1; // If the wind speed has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return -1.
      break;
    }

    serial->write(COM, 8); // Send the command of reading the wind speed
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == address)
      { // Read and judge the packet header.
        Data[0] = ch;
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x03)
          { // Read and judge the packet header.
            Data[1] = ch;
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x02)
              { // Read and judge the packet header.
                Data[2] = ch;
                if (readN(&Data[3], 4) == 4)
                {
                  if (CRC16_2(Data, 5) == ((Data[5] << 8) | Data[6]))
                  {                                         // Check data packet
                    WindSpeed = ((Data[3] << 8) | Data[4]); // Calculate the wind speed
                    ret = true;
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return WindSpeed;
}

int16_t WindFunctions::readWindDirection16(uint8_t address)
{
  uint8_t Data[7] = {0};                                             // Store the original data packet returned by the sensor
  uint8_t COM[8] = {0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00}; // Command of reading wind direction
  boolean ret = false;                                               // Wind direction acquisition success flag
  uint8_t ch = 0;
  COM[0] = address; // Add the complete command package with reference to the communication protocol.
  addedCRC(COM, 6); // Add CRC_16 check for reading wind direction command packet
  int retries = 0;

  while (!ret)
  {
    if (retries > 2)
    {
      WindDirection = 18; // If the wind direction has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return 18.
      break;
    }

    serial->write(COM, 8); // Send the command of reading the wind speed
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == address)
      { // Read and judge the packet header.
        Data[0] = ch;
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x03)
          { // Read and judge the packet header.
            Data[1] = ch;
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x02)
              { // Read and judge the packet header.
                Data[2] = ch;
                if (readN(&Data[3], 4) == 4)
                {
                  if (CRC16_2(Data, 5) == ((Data[5] << 8) | Data[6]))
                  { // Check data packet
                    ret = true;
                    WindDirection = Data[4]; // Calculate the wind direction
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return WindDirection;
}

int16_t WindFunctions::readWindDirection360(uint8_t address)
{
  uint8_t Data[7] = {0};                                             // Store the original data packet returned by the sensor
  uint8_t COM[8] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00}; // Command of reading wind direction
  boolean ret = false;                                               // Wind direction acquisition success flag
  uint8_t ch = 0;
  COM[0] = address; // Add the complete command package with reference to the communication protocol.
  addedCRC(COM, 6); // Add CRC_16 check for reading wind direction command packet
  int retries = 0;

  while (!ret)
  {
    if (retries > 2)
    {
      WindDirection = -1; // If the wind direction has not been read for more than 1000 milliseconds, it will be regarded as a timeout and return 18.
      break;
    }

    serial->write(COM, 8); // Send the command of reading the wind direction
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == address)
      { // Read and judge the packet header.
        Data[0] = ch;
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x03)
          { // Read and judge the packet header.
            Data[1] = ch;
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x02)
              { // Read and judge the packet header.
                Data[2] = ch;
                if (readN(&Data[3], 4) == 4)
                {
                  if (CRC16_2(Data, 5) == ((Data[5] << 8) | Data[6]))
                  { // Check data packet
                    ret = true;
                    WindDirection = (Data[3] << 8) | Data[4]; // Calculate the wind direction by dividing by 10.
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return WindDirection;
}

uint8_t WindFunctions::readAddress()
{
  uint8_t Data[7] = {0};                                             // Store the original data packet returned by the sensor
  uint8_t COM[8] = {0x00, 0x03, 0x10, 0x00, 0x00, 0x01, 0x00, 0x00}; // Command for reading wind speed
  boolean ret = false;                                               // Wind speed acquisition success flag
  uint8_t ch = 0;
  addedCRC(COM, 6); // Add CRC_16 check
  int retries = 0;

  while (!ret)
  {
    if (retries > 2)
    {
      serial->println("Address Read Failed.");
      delay(2000);
      break;
    }
    serial->write(COM, 8); // Send the command of reading the address
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == 0x00)
      { // Read and judge the packet header.
        Data[0] = ch;
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x03)
          { // Read and judge the packet header.
            Data[1] = ch;
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x02)
              { // Read and judge the packet header.
                Data[2] = ch;
                if (readN(&Data[3], 4) == 4)
                {
                  if (CRC16_2(Data, 5) == ((Data[5] << 8) | Data[6]))
                  { // Check data packet
                    ret = true;
                    ReadAddr = Data[4];
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  return ReadAddr;
}

// To modify the sensor address, make sure only one device is powered on and connected. This uses the 0x00 broadcast address.
// Address1: 	The address of the device before modification.
// Address2: 	The modified address of the device, the range is 0x00~0xFF,
// Returns true to indicate that the modification was successful, and returns false to indicate that the modification failed.
// Power the sensor off and on.
boolean WindFunctions::ModifyAddress(uint8_t Address1, uint8_t Address2)
{
  uint8_t ModifyAddressCOM[11] = {0x00, 0x10, 0x10, 0x00, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00};
  boolean ret = false;
  uint8_t ch = 0;
  ModifyAddressCOM[0] = Address1;
  ModifyAddressCOM[8] = Address2;
  addedCRC(ModifyAddressCOM, 9);
  int retries = 0;

  while (!ret)
  {
    if (retries > 2)
    {
      serial->println("Address set failed!");
      delay(2000);
      break;
    }
    serial->write(ModifyAddressCOM, 11);
    serial->flush();
    retries++;

    if (readN(&ch, 1) == 1)
    {
      if (ch == Address1)
      {
        if (readN(&ch, 1) == 1)
        {
          if (ch == 0x10)
          {
            if (readN(&ch, 1) == 1)
            {
              if (ch == 0x10)
              {
                if (readN(&ch, 1) == 1)
                {
                  if (ch == 0x00)
                  {
                    if (readN(&ch, 1) == 1)
                    {
                      if (ch == 0x00)
                      {
                        if (readN(&ch, 1) == 1)
                        {
                          if (ch == 0x01)
                          {
                            while (1)
                            {
                              serial->println("Please power on the sensor again.");
                              delay(2000);
                            }
                            ret = true;
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }
  return ret;
}
