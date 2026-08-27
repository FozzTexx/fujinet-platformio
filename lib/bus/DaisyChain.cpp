#include "DaisyChain.h"

virtualDevice *DaisyChain::deviceWithID(fujiDeviceID_t devID)
{
    auto it = _daisyChain.find(devID);
    if (it != _daisyChain.end())
        return it->second;
    return nullptr;
}

void DaisyChain::addDevice(virtualDevice *newDev, fujiDeviceID_t devID)
{
    newDev->_devnum = devID;
    _daisyChain[devID] = newDev;
    return;
}

void DaisyChain::changeDeviceID(virtualDevice *device, fujiDeviceID_t newID)
{
    for (auto it = _daisyChain.begin(); it != _daisyChain.end(); ++it)
    {
        if (it->second == device)
        {
            auto nodeHandler = _daisyChain.extract(it);
            nodeHandler.key() = newID;
            device->_devnum = (fujiDeviceID_t) newID;
            _daisyChain.insert(std::move(nodeHandler));
            break;
        }
    }

    return;
}
