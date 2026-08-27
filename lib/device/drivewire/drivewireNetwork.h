#ifndef DRIVEWIRENETWORK_H
#define DRIVEWIRENETWORK_H

#include "NDevice.h"

#ifdef UNUSED
class drivewireNetwork : public NDevice
{
 public:
#ifdef HAVE_LAST_ERROR
    nDevStatus_t getErrorCode() override { return lastError; }
    void setErrorCode(nDevStatus_t err) override { lastError = err; }
#endif /* HAVE_LAST_ERROR */
};
#else
using drivewireNetwork = NDevice;
#endif /* UNUSED */

#endif /* DRIVEWIRENETWORK_H */
