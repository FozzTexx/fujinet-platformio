#ifdef BUILD_IEC

#include "disk.h"

#include <cstring>

#include "../../include/debug.h"

#include "fuji.h"
#include "utils.h"

#include "meat_io.h"
#include "meat_buffer.h"

// External ref to fuji object.
extern iecFuji theFuji;

iecDisk::iecDisk()
{
    // device_active = false;
    device_active = true; // temporary during bring-up
}

// Read disk data and send to computer
void iecDisk::read()
{
    // TODO: IMPLEMENT
}

// Write disk data from computer
void iecDisk::write(bool verify)
{
    // TODO: IMPLEMENT
}

// Disk format
void iecDisk::format()
{
    // TODO IMPLEMENT
}

/* Mount Disk
   We determine the type of image based on the filename exteniecn.
   If the disk_type value passed is not MEDIATYPE_UNKNOWN then that's used instead.
   If filename has no extension or is NULL and disk_type is MEDIATYPE_UNKOWN,
   then we assume it's MEDIATYPE_ATR.
   Return value is MEDIATYPE_UNKNOWN in case of failure.
*/
mediatype_t iecDisk::mount(FILE *f, const char *filename, uint32_t disksize, mediatype_t disk_type)
{
    // TODO IMPLEMENT
    return MEDIATYPE_UNKNOWN; // MEDIATYPE_UNKNOWN
}

// Destructor
iecDisk::~iecDisk()
{
    if (_disk != nullptr)
        delete _disk;
}

// Unmount disk file
void iecDisk::unmount()
{
    Debug_print("disk UNMOUNT\n");

    if (_disk != nullptr)
    {
        _disk->unmount();
        device_active = false;
    }
}

// Create blank disk
bool iecDisk::write_blank(FILE *f, uint16_t sectorSize, uint16_t numSectors)
{
    // TODO IMPLEMENT
    return false;
}

void iecDisk::process_load()
{
}

void iecDisk::process_save()
{
}

void iecDisk::process_command()
{
}

void iecDisk::process_file()
{
}

// Process command
device_state_t iecDisk::process(IECData *id)
{
    Debug_printf("iecDisk::process()\n");
    virtualDevice::process(id);

    switch (commanddata->channel)
    {
    case 0: // LOAD
        process_load();
        break;
    case 1: // SAVE
        process_save();
        break;
    case 15: // COMMAND
        process_command();
        break;
    default: // Open files (2-14)
        process_file();
        break;
    }
    return device_state;
}

#endif /* BUILD_ATARI */