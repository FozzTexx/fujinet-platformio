#ifdef BUILD_ADAM

#include "adamFuji.h"
#include "fujiCommandID.h"
#include "fujiDeviceID.h"

#include <cstring>

#include "../../include/debug.h"

#include "serial.h"

#include "fnSystem.h"
#include "fnConfig.h"
#include "fnWiFi.h"
#include "fsFlash.h"
#include "fnio.h"
#include "led.h"

#include "utils.h"
#include "string_utils.h"
#include "fuji_endian.h"
#include "compat_string.h"

#define IMAGE_EXTENSION ".ddp"
#define COPY_SIZE 532

adamFuji platformFuji;
fujiDevice *theFuji = &platformFuji;         // global fuji device object
adamNetwork *theNetwork;  // global network device object (temporary)
adamNetwork *theNetwork2; // another network device
adamPrinter *thePrinter;  // global printer
adamSerial *theSerial;    // global serial

using namespace std;

// sioDisk sioDiskDevs[MAX_HOSTS];
// sioNetwork sioNetDevs[MAX_NETWORK_DEVICES];

// Constructor
adamFuji::adamFuji() : fujiDevice(MAX_DISK_DEVICES, IMAGE_EXTENSION, std::nullopt)
{
    // Helpful for debugging
    for (int i = 0; i < MAX_HOSTS; i++)
        _fnHosts[i].slotid = i;
#ifndef ESP_PLATFORM
    // BoIP: TNFS-backed Fuji ops overrun the 300us window; send anyway. See disk.cpp.
    _pc_no_response_deadline = true;
#endif
}

AdamNetStatus adamFuji::deviceStatus()
{
    AdamNetStatus status;

    status.length = 1024;
    status.devtype = ADAMNET_DEVTYPE::CHAR;
    status.status = 0;

    return status;
}

// DEBUG TAPE
void adamFuji::debug_tape()
{
}

// This gets called when we're about to shutdown/reboot
void adamFuji::shutdown()
{
    for (int i = 0; i < MAX_DISK_DEVICES; i++)
        _fnDisks[i].disk_dev.unmount();
}

size_t adamFuji::set_additional_direntry_details(fsdir_entry_t *f, uint8_t *dest,
                                                 uint8_t maxlen)
{
    struct {
        dirEntryTimestamp modified;
        uint32_t size;
        uint8_t is_dir;
        uint8_t is_trunc;
        uint8_t mediatype;
    } __attribute__((packed)) custom_details;
    dirEntryDetails details;

    details = _additional_direntry_details(f);
    custom_details.modified = details.modified;
    custom_details.modified.year -= 100;
    custom_details.size = htole32(details.size);
    custom_details.is_dir = details.flags & DET_FF_DIR;
    custom_details.mediatype = details.mediatype;

    maxlen -= sizeof(custom_details);
    // Subtract a byte for a terminating slash on directories
    if (custom_details.is_dir)
        maxlen--;

    custom_details.is_trunc = strlen(f->filename) >= maxlen ? DET_FF_TRUNC : 0;
    memcpy(dest, &custom_details, sizeof(custom_details));
    return sizeof(custom_details);
}

//  Make new disk and shove into device slot
void adamFuji::adamnet_new_disk(const FujiAdamPacket &packet)
{
    uint8_t hs = packet.param(0);
    uint8_t ds = packet.param(1);
    u32le_t numBlocks;
    const char *ptr;

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    ptr = packet.dataAsString()->c_str();
    memcpy(&numBlocks, ptr, sizeof(numBlocks));
    ptr += sizeof(numBlocks);

    fujiDisk &disk = _fnDisks[ds];
    fujiHost &host = _fnHosts[hs];

    if (new_disk_completed)
    {
        new_disk_completed = false;
        SYSTEM_BUS.start_time = GET_TIMESTAMP();
        SYSTEM_BUS.transaction_error();
        return;
    }

    disk.host_slot = hs;
    disk.access_mode = DISK_ACCESS_MODE_WRITE;
    strlcpy(disk.filename, (const char *) ptr, sizeof(disk.filename));

    disk.fileh = host.fnfile_open(disk.filename, disk.filename, sizeof(disk.filename), "w");

    Debug_printf("Creating file %s on host slot %u mounting in disk slot %u numblocks: %lu\n", disk.filename, hs, ds, numBlocks);

    disk.disk_dev.write_blank(disk.fileh, numBlocks);

    fnio::fclose(disk.fileh);

    new_disk_completed = true;
    SYSTEM_BUS.transaction_success();
}

// Mounts the desired boot disk number
void adamFuji::insert_boot_device(uint8_t d)
{
    const char *config_atr = "/autorun.ddp";
    const char *mount_all_atr = "/mount-and-boot.ddp";
    fnFile *fBoot;

    switch (d)
    {
    case 0:
        fBoot = fsFlash.fnfile_open(config_atr);
        _fnDisks[0].disk_dev.mount(fBoot, config_atr, 262144, DISK_ACCESS_MODE_READ, MEDIATYPE_DDP);
        break;
    case 1:

        fBoot = fsFlash.fnfile_open(mount_all_atr);
        _fnDisks[0].disk_dev.mount(fBoot, mount_all_atr, 262144, DISK_ACCESS_MODE_READ, MEDIATYPE_DDP);
        break;
    }

    _fnDisks[0].disk_dev.is_config_device = true;
    _fnDisks[0].disk_dev.device_active = true;
}

void adamFuji::adamnet_enable_device(const FujiAdamPacket &packet)
{
    fujiDeviceID_t d = static_cast<fujiDeviceID_t>(packet.param8(0));

    Debug_printf("FUJI ENABLE DEVICE %02x\n", d);

    SYSTEM_BUS.start_time = GET_TIMESTAMP();

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    switch (d)
    {
    case FUJI_DEVICEID_PRINTER:
        Config.store_printer_enabled(true);
        break;
    case FUJI_DEVICEID_DISK:
        Config.store_device_slot_enable_1(true);
        break;
    case FUJI_DEVICEID_DISK2:
        Config.store_device_slot_enable_2(true);
        break;
    case FUJI_DEVICEID_DISK3:
        Config.store_device_slot_enable_3(true);
        break;
    case FUJI_DEVICEID_DISK4:
        Config.store_device_slot_enable_4(true);
        break;
    default:
        break;
    }

    Config.save();

    SYSTEM_BUS.enableDevice(d);
    SYSTEM_BUS.transaction_success();
}

void adamFuji::adamnet_disable_device(const FujiAdamPacket &packet)
{
    fujiDeviceID_t d = static_cast<fujiDeviceID_t>(packet.param8(0));

    Debug_printf("FUJI DISABLE DEVICE %02x\n", d);

    SYSTEM_BUS.start_time = GET_TIMESTAMP();
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    switch (d)
    {
    case FUJI_DEVICEID_PRINTER:
        Config.store_printer_enabled(false);
        break;
    case FUJI_DEVICEID_DISK:
        Config.store_device_slot_enable_1(false);
        break;
    case FUJI_DEVICEID_DISK2:
        Config.store_device_slot_enable_2(false);
        break;
    case FUJI_DEVICEID_DISK3:
        Config.store_device_slot_enable_3(false);
        break;
    case FUJI_DEVICEID_DISK4:
        Config.store_device_slot_enable_4(false);
        break;
    default:
        break;
    }

    Config.save();

    SYSTEM_BUS.disableDevice(d);
    SYSTEM_BUS.transaction_success();
}

// Initializes base settings and adds our devices to the SIO bus
void adamFuji::setup()
{
    // set up Fuji device
    populate_slots_from_config();

    // Disable booting from CONFIG if our settings say to turn it off
    boot_config = Config.get_general_config_enabled();

    // Disable status_wait if our settings say to turn it off
    status_wait_enabled = false;

    SYSTEM_BUS.addDevice(&_fnDisks[0].disk_dev, FUJI_DEVICEID_DISK);
    SYSTEM_BUS.addDevice(&_fnDisks[1].disk_dev, FUJI_DEVICEID_DISK2);
    SYSTEM_BUS.addDevice(&_fnDisks[2].disk_dev, FUJI_DEVICEID_DISK3);
    SYSTEM_BUS.addDevice(&_fnDisks[3].disk_dev, FUJI_DEVICEID_DISK4);

    // Read and enable devices
    _fnDisks[0].disk_dev.device_active = Config.get_device_slot_enable_1();
    _fnDisks[1].disk_dev.device_active = Config.get_device_slot_enable_2();
    _fnDisks[2].disk_dev.device_active = Config.get_device_slot_enable_3();
    _fnDisks[3].disk_dev.device_active = Config.get_device_slot_enable_4();

    if (boot_config == true)
    {
        Debug_printf("Config General Boot Mode: %u\n", Config.get_general_boot_mode());
        if (Config.get_general_boot_mode() == 0)
        {
            fnFile *f = fsFlash.fnfile_open("/autorun.ddp");
            _fnDisks[0].disk_dev.mount(f, "/autorun.ddp", 262144, DISK_ACCESS_MODE_READ, MEDIATYPE_DDP);
            _fnDisks[0].disk_dev.is_config_device = true;
        }
        else
        {
            fnFile *f = fsFlash.fnfile_open("/mount-and-boot.ddp");
            _fnDisks[0].disk_dev.mount(f, "/mount-and-boot.ddp", 262144, DISK_ACCESS_MODE_READ, MEDIATYPE_DDP);
        }
    }
    else
    {
        Debug_printf("Not mounting config disk\n");
    }

    // Create these once, to avoid leaking them when setup() re-runs on an
    // in-process restart.
    if (theNetwork == nullptr)
    {
        theNetwork = new adamNetwork();
        theNetwork2 = new adamNetwork();
        theSerial = new adamSerial();
        SYSTEM_BUS.addDevice(theNetwork, FUJI_DEVICEID_NETWORK);  // temporary.
        SYSTEM_BUS.addDevice(theNetwork2, static_cast<fujiDeviceID_t>(FUJI_DEVICEID_NETWORK + 1)); // temporary
        SYSTEM_BUS.addDevice(theFuji, FUJI_DEVICEID_FUJINET);    // Fuji becomes the gateway device.
    }
}

void adamFuji::adamnet_get_time()
{
    Debug_println("FUJI GET TIME");

    SYSTEM_BUS.start_time = GET_TIMESTAMP();
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    time_t tt = time(nullptr);

    setenv("TZ", Config.get_general_timezone().c_str(), 1);
    tzset();

    struct tm *now = localtime(&tt);

        /*
     NWD order has changed to match apple format
     Previously:
        response[0] = now->tm_mday;
        response[1] = now->tm_mon;
        response[2] = now->tm_year;
        response[3] = now->tm_hour;
        response[4] = now->tm_min;
        response[5] = now->tm_sec;
    */

        response[0] = (now->tm_year) / 100 + 19;
        response[1] = now->tm_year % 100;
        response[2] = now->tm_mon + 1;
        response[3] = now->tm_mday;
        response[4] = now->tm_hour;
        response[5] = now->tm_min;
        response[6] = now->tm_sec;

        response_len = 7;

    Debug_printf("Sending %02X %02X %02X %02X %02X %02X %02X\n", response[0], response[1], response[2], response[3], response[4], response[5], response[6]);
    SYSTEM_BUS.transaction_success();
}

void adamFuji::adamnet_device_enable_status(const FujiAdamPacket &packet)
{
    uint8_t d = packet.param(0);

    SYSTEM_BUS.start_time = GET_TIMESTAMP();
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (SYSTEM_BUS.deviceExists(d))
        SYSTEM_BUS.transaction_success();
    else
        SYSTEM_BUS.transaction_error();

    response_len = 1;
    response[0] = SYSTEM_BUS.deviceEnabled(d);
}

void adamFuji::adamnet_control_send(const FujiAdamPacket &packet)
{
    /*****************************************************************
     * TERRIBLE TERRIBLE backwards compatibility hack! Someone decided
     * on Adam that FUJICMD_SET_DEVICE_FULLPATH only needed one arg
     * instead of three! Do not ever add anything else here. It is
     * wrong and shouldn't have happened in the first place!
     *****************************************************************/
    const FujiAdamPacket *backPacket = &packet;
    std::optional<FujiAdamPacket> compatPacket;

    if (backPacket->command() == FUJICMD_SET_DEVICE_FULLPATH)
    {
        uint8_t deviceSlot = backPacket->param(0);
        const ByteBuffer &data = backPacket->data().value();

        compatPacket.emplace(backPacket->device(), backPacket->type(), backPacket->command(),
                             deviceSlot, _fnDisks[deviceSlot].host_slot,
                             (uint8_t) _fnDisks[deviceSlot].access_mode, data);
        backPacket = &*compatPacket;
    }

    // Let the base class handle standard commands
    if (fujiDevice::processCommand(*backPacket))
        return;

    switch (backPacket->command())
    {
    case FUJICMD_NEW_DISK:
        adamnet_new_disk(*backPacket);
        break;
    case FUJICMD_ENABLE_DEVICE:
        adamnet_enable_device(*backPacket);
        break;
    case FUJICMD_DISABLE_DEVICE:
        adamnet_disable_device(*backPacket);
        break;
    case FUJICMD_GET_TIME:
        adamnet_get_time();
        break;
    case FUJICMD_DEVICE_ENABLE_STATUS:
        adamnet_device_enable_status(*backPacket);
        break;
    default:
        Debug_printv("Unknown command: %02x\n", backPacket->command());
        break;
    }
}

void adamFuji::adamnet_control_clr()
{
    FujiAdamPacket packet(_devnum, APT::NM_SEND,
                          ByteBuffer(response, response + response_len));
    auto encoded = packet.serialize();
    adamnet_send_buffer(encoded.data(), encoded.size());
    response_len = 0;
}

void adamFuji::fujicmd_read_directory_entry(size_t maxlen, uint8_t addtl)
{
    if (response_len)
    {
        // Still have queued up data that needs to be read, ignore this request
        SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
        SYSTEM_BUS.transaction_success();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    Debug_printf("Fuji cmd: READ DIRECTORY ENTRY (max=%hu) (addtl=%02x)\n", maxlen, addtl);

    auto current_entry = fujicore_read_directory_entry(maxlen, addtl);
    if (!current_entry)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    std::string &dirpath = *current_entry;

    // Hack-o-rama to add file type character to beginning of path.
    if (maxlen == 31)
    {
        dirpath.resize(dirpath.size() + 2);
        memmove(&dirpath[2], &dirpath[0], dirpath.size() - 2);
        dirpath[0] = dirpath[1] = 0x20;

        // Check if it's a directory first
        if (dirpath.back() == '/')
        {
            dirpath[0] = 0x83;
            dirpath[1] = 0x84;
        }
        else
        {
            size_t dot_pos = dirpath.rfind('.');
            if (dot_pos != std::string::npos)
            {
                std::string ext = dirpath.substr(dot_pos + 1);
                // Convert to uppercase for comparison
                std::transform(ext.begin(), ext.end(), ext.begin(), ::toupper);

                if (ext == "DDP")
                {
                    dirpath[0] = 0x85;
                    dirpath[1] = 0x86;
                }
                else if (ext == "DSK")
                {
                    dirpath[0] = 0x87;
                    dirpath[1] = 0x88;
                }
                else if (ext == "ROM")
                {
                    dirpath[0] = 0x89;
                    dirpath[1] = 0x8a;
                }
            }
        }
    }

    if (current_entry->size() < maxlen)
        current_entry->resize(maxlen, '\0');

    Debug_printf("%s\n", util_hexdump(current_entry->data(), maxlen).c_str());
    SYSTEM_BUS.transaction_send(current_entry->data(), maxlen);
}

#endif /* BUILD_ADAM */
