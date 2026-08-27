#ifndef DAISYCHAIN_H
#define DAISYCHAIN_H

#include "fujiDeviceID.h"

#include <map>

class virtualDevice;

class DaisyChain
{
protected:
    std::map<fujiDeviceID_t, virtualDevice *> _daisyChain;

public:
    virtualDevice *deviceWithID(fujiDeviceID_t devID);
    void addDevice(virtualDevice *newDev, fujiDeviceID_t devID);
    void changeDeviceID(virtualDevice *device, fujiDeviceID_t newID);

    struct Iterator {
        // 1. Mandatory type aliases for STL algorithms and standard operators
        using iterator_category = std::forward_iterator_tag;
        using value_type        = virtualDevice*;
        using difference_type   = std::ptrdiff_t;
        using pointer           = virtualDevice**;
        using reference         = virtualDevice*&;

        std::map<fujiDeviceID_t, virtualDevice*>::const_iterator map_iter;

        // 2. Dereference operators
        virtualDevice* operator*() const { return map_iter->second; }
        virtualDevice* operator->() const { return map_iter->second; }

        // 3. Increment operators (Prefix and Postfix)
        Iterator& operator++() {
            ++map_iter;
            return *this;
        }
        Iterator operator++(int) {
            Iterator tmp = *this;
            ++map_iter;
            return tmp;
        }

        // 4. Comparison Operators (Now supporting BOTH == and !=)
        bool operator==(const Iterator& other) const { return map_iter == other.map_iter; }
        bool operator!=(const Iterator& other) const { return map_iter != other.map_iter; }
    };

    // Public entry hooks for your container loops
    Iterator begin() const { return Iterator{_daisyChain.cbegin()}; }
    Iterator end()   const { return Iterator{_daisyChain.cend()}; }
};

#endif /* DAISYCHAIN_H */
