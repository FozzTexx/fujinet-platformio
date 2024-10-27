#ifdef BUILD_IEC
#ifndef MODEM_H
#define MODEM_H

#include "bus.h"

#include "../modem-sniffer/modem-sniffer.h"
#include <cstdint>

class iecModem : public virtualDevice
{
private:
    ModemSniffer* modemSniffer;
    FileSystem *activeFS;
    time_t _lasttime;

protected:
    virtual bool openChannel(int chan, IECPayload &payload);
    virtual bool closeChannel(int chan);
    virtual bool readChannel(int chan);
    virtual bool writeChannel(int chan, IECPayload &payload);

public:
    iecModem(FileSystem *_fs, bool snifferEnable);
    virtual ~iecModem();

    ModemSniffer *get_modem_sniffer() { return modemSniffer; }
    time_t get_last_activity_time() { return _lasttime; } // timestamp of last input or output.

};

#endif
#endif
