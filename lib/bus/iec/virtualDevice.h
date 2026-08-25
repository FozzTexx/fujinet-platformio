#ifndef VIRTUALDEVICE_H
#define VIRTUALDEVICE_H

#ifdef BUILD_IEC

#include "IECFileDevice.h"
#include "FujiIECPacket.h"

class virtualDevice : public IECFileDevice
{
protected:
    device_state_t state;

    virtual void shutdown() {};

    virtual bool processCommand(const FujiIECPacket &packet) = 0;

#ifdef UNUSED
    // FIXME - this should all be handled by the bus, not individual devices
    void talk(uint8_t secondary) override;
    void listen(uint8_t secondary) override;
    void untalk() override;
    void unlisten() override;
    int8_t canWrite() override;
    int8_t canRead() override;
    void write(uint8_t data, bool eoi) override;
    uint8_t read() override;
    void task() override;
#endif /* UNUSED */

    // IEC channel methods
    bool open(uint8_t channel, const char *name) override;
    void close(uint8_t channel) override;
    uint8_t read(uint8_t channel, uint8_t *buffer, uint8_t bufferSize, bool *eoi) override;
    uint8_t write(uint8_t channel, uint8_t *buffer, uint8_t bufferSize, bool eoi) override;

 public:
    virtualDevice();
    virtualDevice(uint8_t devnr) : IECFileDevice(devnr) {}
};

#endif /* BUILD_IEC */

#endif /* VIRTUALDEVICE_H */
