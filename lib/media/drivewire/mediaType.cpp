#ifdef BUILD_COCO

#include "mediaType.h"

#include <cstdint>
#include <cstring>

#include "../../include/debug.h"

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
    Debug_printf("DW MediaType baseclass READ file\n");

    return FUJI_ERROR::UNSPECIFIED;
}

fujiError_t MediaType::write(uint32_t blockNum, bool verify)
{
    return FUJI_ERROR::UNSPECIFIED;
}

void MediaType::get_block_buffer(uint8_t **p_buffer, uint16_t *p_blk_size)
{
    *p_buffer = &_media_blockbuff[0];
    *p_blk_size = MEDIA_BLOCK_SIZE;
}

void MediaType::unmount()
{
    if (_media_fileh != nullptr)
    {
        fnio::fclose(_media_fileh);
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
        if (strcasecmp(ext, "DSK") == 0)
        {
            return MEDIATYPE_DSK;
        }
        else if (strcasecmp(ext, "MRM") == 0 || strcasecmp(ext, "RMM") == 0)
        {
            return MEDIATYPE_MRM;
        }
        else if (strcasecmp(ext, "VDK") == 0)
        {
            return MEDIATYPE_VDK;
        }
    }
    return MEDIATYPE_UNKNOWN;
}

#endif // NEW_TARGET
