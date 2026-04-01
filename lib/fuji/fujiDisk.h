#ifndef _FUJI_DISK_
#define _FUJI_DISK_

#include "../device/disk.h"
#include "fujiHost.h"

#define MAX_DISPLAY_FILENAME_LEN 36
#define MAX_FILENAME_LEN 256

#define INVALID_HOST_SLOT 0xFF

class fujiDisk
{
public:
    disk_access_flags_t access_mode = DISK_ACCESS_MODE_READ;
    uint8_t host_slot = INVALID_HOST_SLOT;
    char filename[MAX_FILENAME_LEN] = { '\0' };
#ifdef DISK_ROLES_MIXED
    fnFile* fileh = nullptr;
    mediatype_t disk_type = MEDIATYPE_UNKNOWN;
    uint32_t disk_size = 0;
    fujiHost *host = nullptr;
#endif /* DISK_ROLES_MIXED */
    DISK_DEVICE disk_dev;

    void reset();
    void reset(const char *filename, uint8_t hostslot, disk_access_flags_t access_mode);
};

#endif // _FUJI_DISK_
