#include "ROMFactory.h"
#include "CoCoCartROM.h"

std::unique_ptr<ROMMedia> ROMFactory::open(const std::string &path, fnFile *handle)
{
    // One line per format, in search order:
    if (auto media = Media::open<CoCoCartROM>(path, handle)) return media;

    return nullptr;
}
