/**
 * Network Status object
 */

#ifndef NETWORKSTATUS_H
#define NETWORKSTATUS_H

#include "status_error_codes.h"
#include <cstdint>


class NetworkStatus
{
public:
    NetworkStatus()
    {
        reset();
    }

    /**
     * Not used
     */
    uint8_t connected;

    /**
     * Error code to return to CIO or SIO caller. (1-255)
     */
    networkStatusError_t error;

    /**
     * Reset status
     */
    void reset()
    {
        connected=0;
        error=NETWORK_ERROR_SUCCESS;
    }
};

#endif /* NETWORKSTATUS_H */
