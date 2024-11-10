#include "GwSerialModbusTask.h"
#include "WindFunctions.h"

class GwSerialModbusTaskStream : public Stream
{
private:
    GwSerialModbusTask *serial;
    bool partialWrites;

public:
    GwSerialModbusTaskStream(GwSerialModbusTask *serial, bool partialWrites = false)
    {
        this->serial = serial;
        this->partialWrites = partialWrites;
    }
    virtual int available()
    {
        if (!serial->isInitialized())
            return 0;
        if (!serial->readBuffer)
            return 0;
        return serial->readBuffer->usedSpace();
    }
    virtual int read()
    {
        if (!serial->isInitialized())
            return -1;
        if (!serial->readBuffer)
            return -1;
        return serial->readBuffer->read();
    }
    virtual int peek()
    {
        if (!serial->isInitialized())
            return -1;
        if (!serial->readBuffer)
            return -1;
        return serial->readBuffer->peek();
    }
    virtual void flush() {};
    virtual size_t write(uint8_t v)
    {
        if (!serial->isInitialized())
            return 0;
        size_t rt = serial->buffer->addData(&v, 1, partialWrites);
        return rt;
    }
    virtual size_t write(const uint8_t *buffer, size_t size)
    {
        if (!serial->isInitialized())
            return 0;
        size_t rt = serial->buffer->addData(buffer, size, partialWrites);
        return rt;
    }
};

GwSerialModbusTask::GwSerialModbusTask(GwLog *logger, Stream *s, int id, bool allowRead) : serial(s)
{
    LOG_DEBUG(GwLog::DEBUG, "creating GwSerialModbusTask %p id %d", this, id);
    this->id = id;
    this->logger = logger;
    String bufName = "Ser(";
    bufName += String(id);
    bufName += ")";
    this->buffer = new GwBuffer(logger, GwBuffer::TX_BUFFER_SIZE, bufName + "wr");
    this->allowRead = allowRead;
    if (allowRead)
    {
        this->readBuffer = new GwBuffer(logger, GwBuffer::RX_BUFFER_SIZE, bufName + "rd");
    }
    buffer->reset("init");
    initialized = true;
}
GwSerialModbusTask::~GwSerialModbusTask()
{
    delete buffer;
    if (readBuffer)
        delete readBuffer;
}

bool GwSerialModbusTask::isInitialized() { return initialized; }
size_t GwSerialModbusTask::enqueue(const uint8_t *data, size_t len, bool partial)
{
    if (!isInitialized())
        return 0;
    return buffer->addData(data, len, partial);
}
GwBuffer::WriteStatus GwSerialModbusTask::write()
{
    if (!isInitialized())
        return GwBuffer::ERROR;
    size_t numWrite = serial->availableForWrite();
    size_t rt = buffer->fetchData(numWrite, [](uint8_t *buffer, size_t len, void *p)
                                  { return ((GwSerialModbusTask *)p)->serial->write(buffer, len); }, this);
    if (rt != 0)
    {
        LOG_DEBUG(GwLog::DEBUG + 1, "Serial %d write %d", id, rt);
    }
    return buffer->usedSpace() ? GwBuffer::AGAIN : GwBuffer::OK;
}
size_t GwSerialModbusTask::sendToClients(const char *buf, int sourceId, bool partial)
{
    if (sourceId == id)
        return 0;
    size_t len = strlen(buf);
    size_t enqueued = enqueue((const uint8_t *)buf, len, partial);
    if (enqueued != len && !partial)
    {
        LOG_DEBUG(GwLog::DEBUG, "GwSerialModbusTask overflow on channel %d", id);
        overflows++;
    }
    return enqueued;
}
void GwSerialModbusTask::loop(bool handleRead, bool handleWrite)
{
    write();
    if (!isInitialized())
        return;
    if (!handleRead)
        return;
    size_t available = serial->available();

    if (allowRead)
    {
        size_t rd = readBuffer->fillData(MAX_NMEA0183_MSG_LEN, [](uint8_t *buffer, size_t len, void *p) -> size_t
                                         {
                                             GwSerialModbusTask *task = (GwSerialModbusTask *)p;
                                             Stream *serial = task->serial;

                                             WindFunctions wind(serial, 1);

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
        serial->readBytes(buffer, available);
    }
}
void GwSerialModbusTask::readMessages(GwMessageFetcher *writer)
{
    if (!isInitialized())
        return;
    if (!allowRead)
        return;
    writer->handleBuffer(readBuffer);
}

bool GwSerialModbusTask::flush(long max)
{
    if (!isInitialized())
        return false;
    if (!availableWrite)
    {
        if (serial->availableForWrite() < 1)
        {
            return false;
        }
        availableWrite = true;
    }
    auto start = millis();
    while (millis() < (start + max))
    {
        if (write() != GwBuffer::AGAIN)
            return true;
        vTaskDelay(1);
    }
    availableWrite = (serial->availableForWrite() > 0);
    return false;
}
Stream *GwSerialModbusTask::getStream(bool partialWrite)
{
    return new GwSerialModbusTaskStream(this, partialWrite);
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
