/*
MAJOR REV 3 UPDATE - June 12, 2020
After transitioning the Arduino-ESP components from the project to ESP-IDF equivalent
code, the source files have been copied into a new ESP-IDF project. Code massaging and
testing commences...
*/

#include "debug.h"

#include "fnSystem.h"
#include "fnWiFi.h"
#include "fnFsSD.h"
#include "fnFsSPIF.h"
#include "fnConfig.h"
#include "keys.h"
#include "led.h"
#include "sio.h"
#include "disk.h"
#include "fuji.h"
#include "modem.h"
#include "apetime.h"
#include "voice.h"
#include "httpService.h"
#include "printerlist.h"

#include <esp_system.h>
#include <nvs_flash.h>

#ifdef BOARD_HAS_PSRAM
#include <esp32/spiram.h>
#include <esp32/himem.h>
#endif

#ifdef BLUETOOTH_SUPPORT
#include "bluetooth.h"
#endif

// fnSystem is declared and defined in fnSystem.h/cpp
// fnLedManager is declared and defined in led.h/cpp
// fnKeyManager is declared and defined in keys.h/cpp
// fnHTTPD is declared and defineid in HttpService.h/cpp
sioModem sioR;
sioFuji theFuji;
sioApeTime apeTime;
sioVoice sioV;

#ifdef BLUETOOTH_SUPPORT
BluetoothManager btMgr;
#endif

TaskHandle_t _taskh_main_loop;

/*
* Initial setup
*/
void main_setup()
{
#ifdef DEBUG
    fnUartDebug.begin(DEBUG_SPEED);
    unsigned long startms = fnSystem.millis();
    Debug_printf("\n\n--~--~--~--\nFujiNet PlatformIO Started @ %lu\n", startms);
    Debug_printf("Starting heap: %u\n", fnSystem.get_free_heap_size());
#ifdef BOARD_HAS_PSRAM
    Debug_printf("PsramSize %u\n", fnSystem.get_psram_size());
    Debug_printf("himem phys %u\n", esp_himem_get_phys_size());
    Debug_printf("himem free %u\n", esp_himem_get_free_size());
    Debug_printf("himem reserved %u\n", esp_himem_reserved_area_size());
#endif
#endif
    esp_err_t e = nvs_flash_init();
    if (e == ESP_ERR_NVS_NO_FREE_PAGES || e == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        Debug_println("Erasing flash");
        ESP_ERROR_CHECK(nvs_flash_erase());
        e = nvs_flash_init();
    }
    ESP_ERROR_CHECK(e);

    fnKeyManager.setup();
    fnLedManager.setup();

    fnSPIFFS.start();
    fnSDFAT.start();

    // Load our stored configuration
    Config.load();

    // Set up the WiFi adapter
    fnWiFi.start();

    theFuji.setup(SIO);
    SIO.addDevice(&theFuji, SIO_DEVICEID_FUJINET); // the FUJINET!

    SIO.addDevice(&apeTime, SIO_DEVICEID_APETIME); // APETime

    SIO.addDevice(&sioR, SIO_DEVICEID_RS232); // R:

    // Create a new printer object, setting its output depending on whether we have SD or not
    FileSystem *ptrfs = fnSDFAT.running() ? (FileSystem *)&fnSDFAT : (FileSystem *)&fnSPIFFS;
    sioPrinter::printer_type ptype = Config.get_printer_type(0);
    if (ptype == sioPrinter::printer_type::PRINTER_INVALID)
        ptype = sioPrinter::printer_type::PRINTER_FILE_TRIM;

    Debug_printf("Creating a default printer using %s storage and type %d\n", ptrfs->typestring(), ptype);

    sioPrinter *ptr = new sioPrinter(ptrfs, ptype);
    fnPrinters.set_entry(0, ptr, ptype, Config.get_printer_port(0));

    SIO.addDevice(ptr, SIO_DEVICEID_PRINTER + fnPrinters.get_port(0)); // P:

    SIO.addDevice(&sioV, SIO_DEVICEID_FN_VOICE); // P3:

    Debug_printf("%d devices registered\n", SIO.numDevices());

    // Go setup SIO
    SIO.setup();

#ifdef DEBUG
    Debug_print("SIO Voltage: ");
    Debug_println(fnSystem.get_sio_voltage());
    unsigned long endms = fnSystem.millis();
    Debug_printf("Available heap: %u\nSetup complete @ %lu (%lums)\n", fnSystem.get_free_heap_size(), endms, endms - startms);
#endif
}

void fn_service_loop(void *param)
{
    while (true)
    {
        // We don't have any delays in this loop, so IDLE threads will be starved
        // Shouldn't be a problem, but something to keep in mind...
        // Go service BT if it's active
    #ifdef BLUETOOTH_SUPPORT
        if (btMgr.isActive())
            btMgr.service();
        else
    #endif
            SIO.service();
    }
}

/*
* This is the start/entry point for an ESP-IDF program (must use "C" linkage)
*/
extern "C"
{
    void app_main()
    {
        // Call our setup routine
        main_setup();

        // Create a new high-priority task to handle the main loop
        // This is assigned to CPU1 the WiFi task ends up on CPU0
        #define MAIN_STACKSIZE 4096
        #define MAIN_PRIORITY 10
        #define MAIN_CPUAFFINITY 1
        xTaskCreatePinnedToCore(fn_service_loop, "fnLoop",
            MAIN_STACKSIZE, nullptr, MAIN_PRIORITY, nullptr, MAIN_CPUAFFINITY);

        // Sit here twiddling our thumbs
        while (true)
            vTaskDelay(9000 / portTICK_PERIOD_MS);
    }
}
