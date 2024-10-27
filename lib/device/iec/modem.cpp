#ifdef BUILD_IEC

#include "modem.h"

iecModem::iecModem(FileSystem *_fs, bool snifferEnable)
{
    activeFS = _fs;
    modemSniffer = new ModemSniffer(activeFS, snifferEnable);
}

iecModem::~iecModem()
{
    if (modemSniffer != nullptr)
    {
        delete modemSniffer;
        modemSniffer = nullptr;
    }
}

bool iecModem::openChannel(int chan, IECPayload &payload)
{
  assert(0);
}

bool iecModem::closeChannel(int chan)
{
  assert(0);
}

bool iecModem::readChannel(int chan)
{
  assert(0);
}

bool iecModem::writeChannel(int chan, IECPayload &payload)
{
  assert(0);
}

#if 0
device_state_t iecModem::process()
{
    // TODO IMPLEMENT
    return DEVICE_IDLE;
}
#endif

#endif /* BUILD_IEC */
