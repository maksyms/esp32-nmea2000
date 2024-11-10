#ifndef _GWSERIALMODBUSTASK_H
#define _GWSERIALMODBUSTASK_H
#include "HardwareSerial.h"
#include "GwLog.h"
#include "GwBuffer.h"
#include "GwChannelInterface.h"
#include "NMEA0183Msg.h"
#include "GwSerial.h"

class GwSerialModbusTask : public GwSerial
{
public:
    GwSerialModbusTask(GwLog *logger, Stream *s, int i, int type, bool allowRead = true) : GwSerial(logger, s, i, type, allowRead) {}
    virtual ~GwSerialModbusTask() {}
    virtual void loop(bool handleRead = true, bool handleWrite = true);

private:
    bool createMWVMessage(double windSpeed, double windAngle, tNMEA0183Msg &msg);
};

template <typename T>
class GwSerialModbusImpl : public GwSerialModbusTask
{
private:
    unsigned long lastWritable = 0;
    template <class C>
    void beginImpl(C *s, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1) {}
    void beginImpl(HardwareSerial *s, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1)
    {
        s->begin(baud, config, rxPin, txPin);
    }
    template <class C>
    void setError(C *s, GwLog *logger) {}
    void setError(HardwareSerial *s, GwLog *logger)
    {
        LOG_DEBUG(GwLog::LOG, "enable serial errors for channel %d", id);
        s->onReceiveError([logger, this](hardwareSerial_error_t err)
                          { LOG_DEBUG(GwLog::ERROR, "serial error on id %d: %d", this->id, (int)err); });
    }
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    void beginImpl(HWCDC *s, unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1)
    {
        s->begin(baud);
    }
#endif
    template <class C>
    long getFlushTimeoutImpl(const C *) { return 2000; }
#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    long getFlushTimeoutImpl(HWCDC *) { return 200; }
#endif

    template <class C>
    int availableForWrite(C *c)
    {
        return c->availableForWrite();
    }

#if CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32S3
    /**
     * issue #81
     * workaround for the HWCDC beeing stuck at some point in time
     * with availableForWrite == 0 but the ISR being disabled
     * we simply give a small delay of 100ms for availableForWrite being 0
     * and afterwards retrigger the ISR
     */
    int availableForWrite(HWCDC *c)
    {
        int rt = c->availableForWrite();
        if (rt > 0)
        {
            lastWritable = millis();
            return rt;
        }
        unsigned long now = millis();
        if (now > (lastWritable + USBCDC_RESTART_TIME))
        {
            lastWritable = now;
            if (c->isConnected())
            {
                // this retriggers the ISR
                usb_serial_jtag_ll_ena_intr_mask(USB_SERIAL_JTAG_INTR_SERIAL_IN_EMPTY);
            }
        }
        return rt;
    }
#endif

    T *serial;

protected:
    virtual long getFlushTimeout() override
    {
        return getFlushTimeoutImpl(serial);
    }
    virtual int availableForWrite()
    {
        return availableForWrite(serial);
    }

public:
    GwSerialModbusImpl(GwLog *logger, T *s, int i, int type, bool allowRead = true) : GwSerialModbusTask(logger, s, i, type, allowRead), serial(s) {}
    virtual ~GwSerialModbusImpl() {}
    virtual void begin(unsigned long baud, uint32_t config = SERIAL_8N1, int8_t rxPin = -1, int8_t txPin = -1) override
    {
        beginImpl(serial, baud, config, rxPin, txPin);
        setError(serial, logger);
    };
};

#endif
