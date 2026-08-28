#include "DaisyChain.h"

virtualDevice *DaisyChain::deviceWithFujiID(fujiDeviceID_t devID)
{
    auto it = std::find_if(_daisyChain.begin(), _daisyChain.end(),
                           [devID](const DaisyChainEntry &entry) {
                               return entry.fujiID == devID;
                           });

    return it != _daisyChain.end() ? it->device : nullptr;
}

std::optional<fujiDeviceID_t> DaisyChain::fujiIDForDevice(virtualDevice *device)
{
    auto it = std::find_if(_daisyChain.begin(), _daisyChain.end(),
                           [device](const DaisyChainEntry &entry) {
                               return entry.device == device;
                           });

    return it != _daisyChain.end() ? std::optional<fujiDeviceID_t>(it->fujiID) : std::nullopt;
}

void DaisyChain::addDevice(virtualDevice *newDev, fujiDeviceID_t devID)
{
    _daisyChain.push_back({newDev, devID, std::nullopt});
    return;
}

void DaisyChain::swapDevices(virtualDevice *dev1, virtualDevice *dev2)
{
    abort();
}

void DaisyChain::rotateMountedDisksFirstToLast()
{
    abort();
}

void DaisyChain::rotateMountedDisksLastToFirst()
{
    abort();
}

virtualDevice *DaisyChain::deviceWithBusID(busDeviceID_t busID)
{
    auto it = std::find_if(_daisyChain.begin(), _daisyChain.end(),
                           [busID](const DaisyChainEntry &entry) {
                               return entry.externalID == busID;
                           });

    return it != _daisyChain.end() ? it->device : nullptr;
}

std::optional<DaisyChain::busDeviceID_t> DaisyChain::busIDForDevice(virtualDevice *device)
{
    auto it = std::find_if(_daisyChain.begin(), _daisyChain.end(),
                           [device](const DaisyChainEntry &entry) {
                               return entry.device == device;
                           });

    return it != _daisyChain.end() ? it->externalID : std::nullopt;
}

void DaisyChain::resetAllBusIDs()
{
    for (auto &entry : _daisyChain)
        entry.externalID.reset();
}

void DaisyChain::assignBusIDForDevice(virtualDevice *device, busDeviceID_t busID)
{
    for (auto &entry : _daisyChain)
    {
        if (entry.device == device)
        {
            entry.externalID = busID;
            return;
        }
    }
}

virtualDevice *DaisyChain::firstDeviceWithoutBusID()
{
    for (auto &entry : _daisyChain)
    {
        if (!entry.externalID.has_value())
            return entry.device;
    }

    return nullptr;
}
