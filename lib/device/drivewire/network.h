#ifndef NETWORK_H
#define NETWORK_H

#include "NDevice.h"

class drivewireNetwork : public NDevice
{
public:
    /**
     * Check to see if PROCEED needs to be asserted.
     */
    bool poll_interrupt();
};

#endif /* NETWORK_H */
