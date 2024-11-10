#include "GwSerialModbusTask.h"
#include "WindFunctions.h"

void GwSerialModbusTask::loop(bool handleRead, bool handleWrite)
{
    write();
    if (!isInitialized())
        return;
    if (!handleRead)
        return;
    size_t available = stream->available();

    if (allowRead)
    {
        size_t rd = readBuffer->fillData(MAX_NMEA0183_MSG_LEN, [](uint8_t *buffer, size_t len, void *p) -> size_t
                                         {
                                             GwSerialModbusTask *task = (GwSerialModbusTask *)p;
                                             Stream *stream = task->stream;

                                             WindFunctions wind(stream, 1);

                                             // Read wind speed and direction using Modbus
                                             if (wind.readAll())
                                             {
                                                 int16_t speed = wind.WindSpeed;
                                                 int16_t direction = wind.WindAngle;
                                                 // task->logger->logDebug(GwLog::DEBUG, "GwSerialModbusTask speed=%d, direction=%d", speed, direction);
                                                 if (speed >= 0 && direction >= 0)
                                                 {
                                                     // Convert direction from 0-3600 to 0-359
                                                     double windAngle = direction / 10.0;
                                                     // Speed is in m/s
                                                     double windSpeed = speed;

                                                     tNMEA0183Msg msg;
                                                     if (task->createMWVMessage(windSpeed, windAngle, msg))
                                                     {
                                                         // Copy MWV message to buffer
                                                         char msgStr[MAX_NMEA0183_MSG_LEN + 3];
                                                         size_t msgLen = 0;
                                                         if (msg.GetMessage(msgStr, MAX_NMEA0183_MSG_LEN))
                                                         {
                                                             msgLen = strlen(msgStr);
                                                             msgStr[msgLen] = 0x0d;
                                                             msgStr[msgLen + 1] = 0x0a;
                                                             msgStr[msgLen + 2] = 0;
                                                             msgLen += 2;
                                                             if (msgLen > len)
                                                                 msgLen = len;
                                                             memcpy(buffer, msgStr, msgLen);
                                                             // task->logger->logDebug(GwLog::DEBUG, "NMEA0183 message length %d is %s", msgLen, msgStr);
                                                             // task->logger->logDebug(GwLog::DEBUG, "NMEA0183 message %02X %02X %02X", msgStr[msgLen - 3], msgStr[msgLen - 2], msgStr[msgLen - 1]);
                                                         }

                                                         return msgLen;
                                                     }
                                                 }
                                             }

                                             return 0; // Return 0 if reading failed or message couldn't be created
                                         },
                                         this);
        // LOG_DEBUG(GwLog::DEBUG, "GwSerialModbusTask %d read %d bytes", id, rd);
    }
    else if (available > 0)
    {
        uint8_t buffer[10];
        if (available > 10)
            available = 10;
        stream->readBytes(buffer, available);
    }
}

bool GwSerialModbusTask::createMWVMessage(double windSpeed, double windAngle, tNMEA0183Msg &msg)
{
    if (!isInitialized())
        return false;

    // Initialize MWV message (Wind Speed and Angle)
    msg.Init("MWV", "WI");

    // Add wind angle (0-359 degrees)
    msg.AddDoubleField(windAngle);

    // Reference: R = Relative, T = True
    msg.AddStrField("R");

    // Add wind speed
    msg.AddDoubleField(windSpeed);

    // Add units (N = Knots, K = km/h, M = m/s)
    msg.AddStrField("M");

    // Add status (A = Valid)
    msg.AddStrField("A");

    return true;
}
