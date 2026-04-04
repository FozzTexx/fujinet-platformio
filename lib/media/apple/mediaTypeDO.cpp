#ifdef BUILD_APPLE

#include "mediaTypeDO.h"

#include <cstring>
#include "utils.h"
#include "../../include/debug.h"

#define BYTES_PER_BLOCK 512
#define BLOCKS_PER_TRACK 8
#define BYTES_PER_SECTOR 256
#define BYTES_PER_TRACK 4096

// .DO (DOS ordered) disk images are stored in DOS 3.3 logical sector order.
// Each 512 byte ProDOS block is stored as two 256 byte DOS sectors.
// see https://retrocomputing.stackexchange.com/questions/15056/converting-apple-ii-prodos-blocks-to-dos-tracks-and-sectors

// This table maps ProDOS blocks to pairs of DOS logical sectors.
static const int prodos2dos[8][2] = { {0, 14}, {13, 12}, {11, 10}, {9, 8}, {7, 6}, {5, 4}, {3, 2}, {1, 15} };

fujiError_t MediaTypeDO::read(uint32_t blockNum, uint16_t *count, uint8_t* buffer)
{
    uint32_t track = blockNum / BLOCKS_PER_TRACK;
    const int* sectors = prodos2dos[blockNum % BLOCKS_PER_TRACK];

    fujiError_t err = read_sector(track, sectors[0], buffer);

    if (err == FUJI_ERROR::NONE)
        err = read_sector(track, sectors[1], &buffer[BYTES_PER_SECTOR]);

    return err;
}

fujiError_t MediaTypeDO::read_sector(int track, int sector, uint8_t* buffer)
{
    Debug_printf("\r\nMediaTypeDO read track %d sector %d", track, sector);
    
    uint32_t offset = (track * BYTES_PER_TRACK) + (sector * BYTES_PER_SECTOR);

    fujiError_t err = fnio::fseek(_media_fileh, offset, SEEK_SET) != 0
        ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;

    if (err == FUJI_ERROR::NONE)
        err = fnio::fread(buffer, 1, BYTES_PER_SECTOR, _media_fileh) != BYTES_PER_SECTOR
            ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;

    return err;
}

fujiError_t MediaTypeDO::write(uint32_t blockNum, uint16_t *count, uint8_t* buffer)
{
    // Return an error if we're trying to write beyond the end of the disk
    if (blockNum >= num_blocks)
    {
        Debug_printf("\r\nwrite block BEYOND END %lu > %lu", blockNum, num_blocks);
        return FUJI_ERROR::UNSPECIFIED;
    }

    uint32_t track = blockNum / BLOCKS_PER_TRACK;
    const int* sectors = prodos2dos[blockNum % BLOCKS_PER_TRACK];

    fujiError_t err = write_sector(track, sectors[0], buffer);

    if (err == FUJI_ERROR::NONE)
        err = write_sector(track, sectors[1], &buffer[BYTES_PER_SECTOR]);

    return err;
}

fujiError_t MediaTypeDO::write_sector(int track, int sector, uint8_t* buffer)
{
    Debug_printf("\r\nMediaTypeDO write track %d sector %d", track, sector);

    uint32_t offset = (track * BYTES_PER_TRACK) + (sector * BYTES_PER_SECTOR);

    fujiError_t err = fnio::fseek(_media_fileh, offset, SEEK_SET) != 0
        ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;

    if (err == FUJI_ERROR::NONE)
        err = fnio::fwrite(buffer, 1, BYTES_PER_SECTOR, _media_fileh) != BYTES_PER_SECTOR
            ? FUJI_ERROR::UNSPECIFIED : FUJI_ERROR::NONE;

    return err;
}

fujiError_t MediaTypeDO::format(uint16_t *responsesize)
{
    return FUJI_ERROR::UNSPECIFIED;
}

mediatype_t MediaTypeDO::mount(fnFile *f, uint32_t disksize)
{
    switch (disksize) {
        case 35 * BYTES_PER_TRACK:
        case 36 * BYTES_PER_TRACK:
        case 40 * BYTES_PER_TRACK:
            // 35, 36, and 40 tracks are supported (same as Applesauce)
            break;
        default:
	        Debug_printf("\nMediaTypeDO error: unsupported disk image size %ld", disksize);
            return MEDIATYPE_UNKNOWN;
    }

    diskiiemulation = false;
    _media_fileh = f;
    num_blocks = disksize / BYTES_PER_BLOCK;
    return MEDIATYPE_DO;
}

#endif // BUILD_APPLE
