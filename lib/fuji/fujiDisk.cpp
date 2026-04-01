#include "fujiDisk.h"

void fujiDisk::reset()
{
#ifdef DISK_DEVICE
    host_slot = INVALID_HOST_SLOT;
    access_mode = DISK_ACCESS_MODE_READ;
#ifdef DISK_ROLES_MIXED
    filename[0] = '\0';
    fileh = nullptr;
    disk_type = MEDIATYPE_UNKNOWN;
    host = nullptr;
#endif /* DISK_ROLES_MIXED */
#endif
}

void fujiDisk::reset(const char *fname, uint8_t hostslot, disk_access_flags_t mode)
{
#ifdef DISK_DEVICE
    host_slot = hostslot;
    access_mode = mode;
#ifdef DISK_ROLES_MIXED
    fileh = nullptr;
    disk_type = MEDIATYPE_UNKNOWN;
    host = nullptr;
#endif /* DISK_ROLES_MIXED */
#endif
}
