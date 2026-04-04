#ifdef BUILD_LYNX

#include "mediaTypeROM.h"

#include <cstdint>
#include <cstring>

#include "../../include/debug.h"


// Returns byte offset of given sector number
uint32_t MediaTypeROM::_block_to_offset(uint32_t blockNum)
{
    return blockNum * MEDIA_BLOCK_SIZE;
}

// Returns FUJI_ERROR::UNSPECIFIED if an error condition occurred
fujiError_t MediaTypeROM::read(uint32_t blockNum, uint16_t *readcount)
{
    if (blockNum == _media_last_block)
        return FUJI_ERROR::NONE; // We already have block.

    Debug_print("**DISK READ START**\r\n");

    // Return an error if we're trying to read beyond the end of the disk
    if (blockNum > _media_num_blocks-1)
    {
        Debug_printf("::read block %lu > %lu\r\n", blockNum, _media_num_blocks);
        _media_controller_status=2;
        return FUJI_ERROR::UNSPECIFIED;
    }

    memset(_media_blockbuff, 0, sizeof(_media_blockbuff));

    fujiError_t err = FUJI_ERROR::NONE;
    // // Perform a seek if we're not reading the sector after the last one we read
    if (blockNum != _media_last_block + 1)
    {
        uint32_t offset = _block_to_offset(blockNum);
        err = fseek(_media_fileh, offset, SEEK_SET) != 0
            ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;
        _media_last_block = INVALID_SECTOR_VALUE;
    }

    if (err == FUJI_ERROR::NONE)
    {
        // handle potential last block partial read
        err = fread(_media_blockbuff, 1, MEDIA_BLOCK_SIZE, _media_fileh) == 0
            ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;
    }

    if (err == FUJI_ERROR::NONE)
    {
        _media_last_block = blockNum;
        _media_controller_status = 0;
        return FUJI_ERROR::NONE;
    }
    else
    {
        _media_last_block = INVALID_SECTOR_VALUE;
        _media_controller_status = 2;
        return FUJI_ERROR::UNSPECIFIED;
    }

    return err;
}

// Returns FUJI_ERROR::UNSPECIFIED if an error condition occurred
fujiError_t MediaTypeROM::write(uint32_t blockNum, bool verify)
{
    return FUJI_ERROR::UNSPECIFIED;
}

uint8_t MediaTypeROM::status()
{
    return _media_controller_status;
}

// Returns FUJI_ERROR::UNSPECIFIED if an error condition occurred
fujiError_t MediaTypeROM::format(uint16_t *responsesize)
{
    return FUJI_ERROR::UNSPECIFIED;
}

mediatype_t MediaTypeROM::mount(FILE *f, uint32_t disksize)
{
    Debug_print("**FILE MOUNT**\r\n");

    _media_fileh = f;
    _mediatype = MEDIATYPE_ROM;
    _media_num_blocks = disksize / MEDIA_BLOCK_SIZE;
    if (disksize % MEDIA_BLOCK_SIZE)       // handle extra bytes
        _media_num_blocks++;

    return _mediatype;
}

// Returns FUJI_ERROR::UNSPECIFIED on error
fujiError_t MediaTypeROM::create(FILE *f, uint32_t numBlocks)
{
    Debug_print("FILE CREATE\r\n");

    return FUJI_ERROR::UNSPECIFIED;
}


#endif /* BUILD_LYNX */
