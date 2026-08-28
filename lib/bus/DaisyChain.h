#ifndef DAISYCHAIN_H
#define DAISYCHAIN_H

#include "fujiDeviceID.h"

#include <optional>
#include <vector>

class virtualDevice;

class DaisyChain
{
public:
    using busDeviceID_t = unsigned;

protected:
    struct DaisyChainEntry {
        virtualDevice *device;
        fujiDeviceID_t fujiID;
        std::optional<busDeviceID_t> externalID;
        bool participatesInBusIDAssignment;
    };

    std::vector<DaisyChainEntry> _daisyChain;

public:
    virtualDevice *deviceWithFujiID(fujiDeviceID_t fujiID);
    std::optional<fujiDeviceID_t> fujiIDForDevice(virtualDevice *device);

    void addDevice(virtualDevice *newDev, fujiDeviceID_t fujiID, bool assignBusID=true);
    void swapDevices(virtualDevice *dev1, virtualDevice *dev2);
    void rotateMountedDisksFirstToLast();
    void rotateMountedDisksLastToFirst();

    virtualDevice *deviceWithBusID(busDeviceID_t busID);
    std::optional<busDeviceID_t> busIDForDevice(virtualDevice *device);
    void resetAllBusIDs();
    void assignBusIDToDevice(virtualDevice *device, busDeviceID_t busID);
    virtualDevice *firstDeviceWithoutBusID();

    class iterator {
        friend class DaisyChain;

        using InternalIterator = std::vector<DaisyChainEntry>::iterator;

        InternalIterator _it;
        iterator(InternalIterator it) : _it(it) {}

    public:
        virtualDevice *operator*() const {
            return _it->device;
        }

        iterator &operator++() {
            ++_it;
            return *this;
        }

        bool operator!=(const iterator &other) const {
            return _it != other._it;
        }
    };

    iterator begin() {
        return iterator(_daisyChain.begin());
    }

    iterator end() {
        return iterator(_daisyChain.end());
    }
};

#endif /* DAISYCHAIN_H */
