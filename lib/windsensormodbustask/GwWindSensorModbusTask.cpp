#include "GwWindSensorModbusTask.h"
#include "GwHardware.h"
#include "WindFunctions.h"
#include "NMEA0183Messages.h"
#include "N2kMessages.h"

// Select serial port for wind sensor communication.
// Can be overridden with -D GWWINDSENSOR_SERIAL=1 or =2 in build flags.
#ifdef GWWINDSENSOR_SERIAL
  #if GWWINDSENSOR_SERIAL == 1
    #define WIND_SERIAL Serial1
  #elif GWWINDSENSOR_SERIAL == 2
    #define WIND_SERIAL Serial2
  #else
    #error "GWWINDSENSOR_SERIAL must be 1 or 2"
  #endif
#else
  // Auto-select: use first serial port not claimed by the core
  #if defined(GWSERIAL_TYPE) || defined(GWSERIAL_MODE)
    #if defined(GWSERIAL2_TYPE) || defined(GWSERIAL2_MODE)
      #error "Both serial ports are already used by the core - set GWWINDSENSOR_SERIAL explicitly"
    #else
      #define WIND_SERIAL Serial2
    #endif
  #else
    #define WIND_SERIAL Serial1
  #endif
#endif

void initWindSensorModbusTask(GwApi *api)
{
  GwLog *logger = api->getLogger();
  LOG_DEBUG(GwLog::LOG, "windsensormodbustask initialized");
  api->addUserTask(runWindSensorModbusTask, "windsensormodbustask", 4000);
}

void runWindSensorModbusTask(GwApi *api)
{
  GwLog *logger = api->getLogger();
  GwConfigHandler *config = api->getConfig();
  LOG_DEBUG(GwLog::LOG, "windsensor task started");

  // TODO: Make sure to use the configured COM port, baud rate, address, RX and TX pins, etc.
  WindFunctions wind(&WIND_SERIAL, 1);
  wind.begin();

  while (true)
  {
    LOG_DEBUG(GwLog::LOG, "Reading wind sensor");
    if (wind.readAll())
    {
      LOG_DEBUG(GwLog::LOG, "Wind speed: %d", wind.WindSpeed);
      LOG_DEBUG(GwLog::LOG, "Wind scale: %d", wind.WindScale);
      LOG_DEBUG(GwLog::LOG, "Wind angle: %d", wind.WindAngle);
      LOG_DEBUG(GwLog::LOG, "Wind direction: %s", wind.getWindDirection());

      tNMEA0183Msg msg;
      // WindAngle is in 0.1 degree units, WindSpeed is in 0.1 m/s units
      // NMEA0183SetMWV expects degrees and m/s, sends unit "M" (m/s)
      NMEA0183SetMWV(msg, wind.WindAngle / 10.0, NMEA0183Wind_Apparent, wind.WindSpeed / 10.0, "WI");
      api->sendNMEA0183Message(msg);
      /*       tN2kMsg n2kMsg;
            SetN2kWindSpeed(n2kMsg, 15, wind.WindSpeed / 10.0, wind.WindAngle / 10.0, tN2kWindReference::N2kWind_Apparent);
            api->sendN2kMessage(n2kMsg);
       */
    }
    vTaskDelay(1000);
  }
  vTaskDelete(NULL);
}
