#ifndef IECCPM_H
#define IECCPM_H

#include "../../bus/bus.h"

#define FOLDERCHAR '/'

// Silly typedefs that runcpm uses
typedef unsigned char   uint8;
typedef unsigned short  uint16;
typedef unsigned int    uint32;

class iecCpm : public virtualDevice
{
    public:

    /**
     * @brief CTOR
     */
    iecCpm();

    /**
     * @brief DTOR
     */
    ~iecCpm();

    protected:
    virtual bool openChannel(int chan, IECPayload &payload);
    virtual bool closeChannel(int chan);
    virtual bool readChannel(int chan);
    virtual bool writeChannel(int chan, IECPayload &payload);

    private:

    TaskHandle_t cpmTaskHandle = NULL;    
};

#endif /* IECCPM_H */
