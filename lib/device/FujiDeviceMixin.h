#ifndef FUJIDEVICEMIXIN_H
#define FUJIDEVICEMIXIN_H

#include "bus.h"

class FujiDeviceMixin : public virtual virtualDevice
{
public:
    virtual bool processCommand(PROCESS_COMMAND_TYPE command) { return false; }
};

#endif /* FUJIDEVICEMIXIN_H */
