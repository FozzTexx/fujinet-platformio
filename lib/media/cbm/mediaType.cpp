#ifdef BUILD_IEC

#include "mediaType.h"

#include <cstring>


MediaType::~MediaType()
{
    unmount();
}

fujiError_t MediaType::format(uint16_t *responsesize)
{
    return FUJI_ERROR::UNSPECIFIED;
}

fujiError_t MediaType::read(uint32_t blockNum, uint16_t *readcount)
{
    return FUJI_ERROR::UNSPECIFIED;
}

fujiError_t MediaType::write(uint32_t blockNum, bool verify)
{
    return FUJI_ERROR::UNSPECIFIED;
}

void MediaType::unmount()
{
    if (_media_fileh != nullptr)
    {
        fclose(_media_fileh);
        _media_fileh = nullptr;
    }
}

mediatype_t MediaType::discover_mediatype(const char *filename)
{
    int l = strlen(filename);
    if (l > 4 && filename[l - 4] == '.')
    {
        // Check the last 3 characters of the string
        const char *ext = filename + l - 3;
        if (strcasecmp(ext, "PRG") == 0)
        {
            return MEDIATYPE_PRG;
        }
        else if (strcasecmp(ext, "D64") == 0)
        {
            return MEDIATYPE_D64;
        }
        else if (strcasecmp(ext, "D71") == 0)
        {
            return MEDIATYPE_D71;
        }
        else if (strcasecmp(ext, "D81") == 0)
        {
            return MEDIATYPE_D81;
        }
    }
    return MEDIATYPE_UNKNOWN;
}

#endif /* BUILD_IEC */
