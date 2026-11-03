#ifndef ROMFACTORY_H
#define ROMFACTORY_H

#include "ROMMedia.h"

#include <memory>
#include <string>

class ROMFactory
{
public:
    ROMFactory() = delete;

    // Returns the first format whose attach() accepts path, or nullptr.
    static std::unique_ptr<ROMMedia> open(const std::string &path, fnFile *handle);
};

#endif /* ROMFACTORY_H */
