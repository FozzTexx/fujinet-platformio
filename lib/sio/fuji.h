#ifndef FUJI_H
#define FUJI_H
#include <cstdint>

#include "../../include/debug.h"
#include "sio.h"
#include "network.h"

#include "fujiHost.h"
#include "fujiDisk.h"

#define MAX_HOSTS 8
#define MAX_DISK_DEVICES 8
#define MAX_NETWORK_DEVICES 8

#define MAX_SSID_LEN 32
#define MAX_WIFI_PASS_LEN 64

class sioFuji : public sioDevice
{
private:

    sioBus *_sio_bus;

    fujiHost _fnHosts[MAX_HOSTS];

    fujiDisk _fnDisks[MAX_DISK_DEVICES];

    int _current_open_directory_slot = -1;

    FILE * atrConfig;     // autorun.atr for FujiNet configuration
    sioDisk configDisk; // special disk drive just for configuration

    uint8_t _countScannedSSIDs = 0;

    void _populate_slots_from_config();
    void _populate_config_from_slots();

protected:

    void sio_reset_fujinet();             // 0xFF
    void sio_net_get_ssid();              // 0xFE
    void sio_net_scan_networks();         // 0xFD
    void sio_net_scan_result();           // 0xFC
    void sio_net_set_ssid();              // 0xFB
    void sio_net_get_wifi_status();       // 0xFA
    void sio_mount_host();                // 0xF9
    void sio_disk_image_mount();          // 0xF8
    void sio_open_directory();            // 0xF7
    void sio_read_directory_entry();      // 0xF6
    void sio_close_directory();           // 0xF5
    void sio_read_host_slots();          // 0xF4
    void sio_write_host_slots();         // 0xF3
    void sio_read_device_slots();         // 0xF2
    void sio_write_device_slots();        // 0xF1
    void sio_disk_image_umount();         // 0xE9
    void sio_get_adapter_config();        // 0xE8
    void sio_new_disk();                  // 0xE7
    void sio_unmount_host();              // 0xE6
    void sio_get_directory_position();    // 0xE5
    void sio_set_directory_position();    // 0xE4

    void sio_status() override;
    void sio_process() override;

    void shutdown() override;

public:
    bool load_config = true;
    sioDisk *disk();
    sioNetwork *network();
    void setup(sioBus *siobus);
    void image_rotate();
    sioFuji();
};

#endif // FUJI_H
