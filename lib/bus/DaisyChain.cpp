#include "DaisyChain.h"

#include "debug.h"

DaisyChain::DaisyChainEntry *DaisyChain::entryForDevice(virtualDevice *device)
{
    auto it = std::find_if(_daisyChain.begin(), _daisyChain.end(),
                           [device](const DaisyChainEntry &entry) {
                               return entry.device == device;
                           });

    return it != _daisyChain.end() ? &*it : nullptr;
}

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
    auto entry = entryForDevice(device);
    return entry ? std::optional<fujiDeviceID_t>(entry->fujiID) : std::nullopt;
}

void DaisyChain::addDevice(virtualDevice *newDev, fujiDeviceID_t devID, bool assignBusID)
{
    _daisyChain.push_back({newDev, devID, std::nullopt, assignBusID});
    return;
}

void DaisyChain::rotate(const std::vector<virtualDevice *> &devices, int amount)
{
    size_t idx, len;

    if (devices.size() < 2 || amount == 0)
        return;

    std::vector<DaisyChainEntry *> entries;

    for (auto *device : devices)
    {
        if (auto *entry = entryForDevice(device))
            entries.push_back(entry);
    }

    len = entries.size();
    if (len != devices.size())
        return;

    amount %= len;

    if (amount < 0)
        amount += len;

    std::vector<virtualDevice *> rotated(len);
    for (idx = 0; idx < len; idx++)
        rotated[(idx + amount) % len] = entries[idx]->device;

    for (idx = 0; idx < len; idx++)
        entries[idx]->device = rotated[idx];
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
    auto entry = entryForDevice(device);
    return entry ? entry->externalID : std::nullopt;
}

void DaisyChain::resetAllBusIDs()
{
    for (auto &entry : _daisyChain)
        entry.externalID.reset();
}

void DaisyChain::assignBusIDToDevice(virtualDevice *device, busDeviceID_t busID)
{
    auto entry = entryForDevice(device);

    if (entry)
        entry->externalID = busID;
}

virtualDevice *DaisyChain::firstDeviceWithoutBusID()
{
    for (auto &entry : _daisyChain)
    {
        if (entry.participatesInBusIDAssignment && !entry.externalID.has_value())
            return entry.device;
    }

    Debug_printf("DAISY CHAIN END\n");
    return nullptr;
}
