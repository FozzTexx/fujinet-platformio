#ifndef IECCLOCK_H
#define IECCLOCK_H

#include "../../bus/bus.h"

#define TC_SIZE 256 // size of returned time string.

class iecClock : public virtualDevice
{
    private:
    
    time_t ts;
    std::string tf;

    protected:
    virtual bool openChannel(int chan, IECPayload &payload);
    virtual bool closeChannel(int chan);
    virtual bool readChannel(int chan);
    virtual bool writeChannel(int chan, IECPayload &payload);

    public:

    iecClock();
    ~iecClock();

    void set_timestamp(std::string s);
    void set_timestamp_format(std::string s);

};

#endif /* IECCLOCK_H */
