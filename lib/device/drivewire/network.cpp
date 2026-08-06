#ifdef BUILD_COCO

#include "network.h"

/**
 * Check to see if PROCEED needs to be asserted, and assert if needed
 * (continue toggling PROCEED).
 */
bool drivewireNetwork::poll_interrupt()
{
    if (!protocol)
        return false;
    uint32_t now = GET_TIMESTAMP();
    if (now - readAck < 5000)
        return false;
    return protocol->available() > 0;
}

#endif /* BUILD_COCO */
