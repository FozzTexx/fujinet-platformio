#ifndef IEC_H
#define IEC_H

// This code uses code from the Meatloaf Project:
// Meatloaf - A Commodore 64/128 multi-device emulator
// https://github.com/idolpx/meatloaf
// Copyright(C) 2020 James Johnston
//
// Meatloaf is free software : you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// Meatloaf is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with Meatloaf. If not, see <http://www.gnu.org/licenses/>.

//
// https://www.pagetable.com/?p=1135
// https://pagetable.com/c64ref/c64disasm/#ED40
// http://unusedino.de/ec64/technical/misc/c1541/romlisting.html#E85B
// https://eden.mose.org.uk/gitweb/?p=rom-reverse.git;a=blob;f=src/vic-1541-sfd.asm;hb=HEAD
// https://www.pagetable.com/docs/Inside%20Commodore%20DOS.pdf
// http://www.ffd2.com/fridge/docs/1541dis.html#E853
// http://unusedino.de/ec64/technical/aay/c1541/
// http://unusedino.de/ec64/technical/aay/c1581/
// http://www.bitcity.de/1541%20Serial%20Interface.htm
// http://www.bitcity.de/theory.htm
// https://comp.sys.cbm.narkive.com/ebz1uFEx/annc-vip-the-virtual-iec-peripheral
// https://www.djupdal.org/cbm/iecata/
// https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/dedic_gpio.html
// https://esp32.com/viewtopic.php?t=27963
// https://github.com/alwint3r/esp32-read-multiple-gpio-pins
// https://github.com/alwint3r/esp32-set-clear-multiple-gpio
// https://www.commodore.ca/wp-content/uploads/2018/11/Commodore-IEC-Serial-Bus-Manual-C64-Plus4.txt
//

#include <cstdint>
#include <string>
#include <vector>
#include <queue>
#include <forward_list>
#include <optional>

#include "iec_ll.h"

#define BUS_DEVICEID_PRINTER 4
#define BUS_DEVICEID_DISK 8
#define BUS_DEVICEID_NETWORK 16

/**
 * @brief The command frame
 */
union cmdFrame_t
{
    struct
    {
        uint8_t device;
        uint8_t comnd;
        uint8_t aux1;
        uint8_t aux2;
        uint8_t cksum;
    };
    struct
    {
        uint32_t commanddata;
        uint8_t checksum;
    } __attribute__((packed));
};

// Fancy wrapper to handle converting the bytes to
// strings, comma separated lists, etc.
class IECPayload
{
private:
  std::vector<uint8_t> _data;
  mutable std::optional<std::string> _dataString;

public:
  IECPayload(const uint8_t *buf, size_t len)
    : _data(buf, buf + len) {}
  const std::string &string() const;
  std::vector<std::string> tokenize(char c=' ') const;
  const uint8_t *data() const;
  size_t size() const;
  const uint8_t &operator[](size_t index) const;
  size_t find(const std::string &str, size_t pos=0) const;
};

class IECRecord
{
public:
  IECCommand_t command;
  IECCommand_t subCommand;
  uint8_t channel;
  bool eoi;
  IECPayload payload;

  IECRecord(uint8_t cmd, uint8_t chan, bool eoiFlag, const uint8_t *buf, size_t len)
    : command((IECCommand_t) (cmd & 0xe0)), subCommand((IECCommand_t) (chan & 0xf0)),
      channel(chan & 0x0f), eoi(eoiFlag), payload(buf, len) {}

  void debugPrint() const;
};

/**
 * @class Forward declaration of System Bus
 */
class systemBus;

/**
 * @class virtualDevice
 * @brief All #FujiNet devices derive from this.
 */
class virtualDevice
{
private:

protected:
    friend systemBus; /* Because we connect to it. */

    /**
     * @brief The device number (ID)
     */
    int _devnum;

    /**
     * @brief The status information to send back on cmd input
     * @param error = the latest error status
     * @param msg = most recent status message
     * @param connected = is most recent channel connected?
     * @param channel = channel of most recent status msg.
     */
    struct _iecStatus
    {
        int8_t error;
        uint8_t cmd;
        std::string msg;
        bool connected;
        int channel;
    } iecStatus;

    /**
     * @brief If response queue is empty, Return 1 if ANY receive buffer has data in it, else 0
     */
    virtual void iec_talk_command_buffer_status() {}

    virtual bool openChannel(int chan, IECPayload &payload) = 0;
    virtual bool closeChannel(int chan) = 0;
    virtual bool readChannel(int chan) = 0;
    virtual bool writeChannel(int chan, IECPayload &payload) = 0;

    // Optional shutdown/reboot cleanup routine
    virtual void shutdown(){};

public:
    /**
     * @brief get the IEC device Number (1-31)
     * @return The device number registered for this device
     */
    int id() { return _devnum; };

    /**
     * @brief Is this virtualDevice holding the virtual disk drive used to boot CONFIG?
     */
    bool is_config_device = false;

    /**
     * @brief is device active (turned on?)
     */
    bool device_active = true;

    void set_iec_status(int8_t error, uint8_t cmd, const std::string msg, bool connected, int channel) {
        iecStatus.error = error;
        iecStatus.cmd = cmd;
        iecStatus.msg = msg;
        iecStatus.connected = connected;
        iecStatus.channel = channel;
    }

    // TODO: does this need to translate the message to PETSCII?
    std::vector<uint8_t> iec_status_to_vector() {
        std::vector<uint8_t> data;
        data.push_back(static_cast<uint8_t>(iecStatus.error));
        data.push_back(iecStatus.cmd);
        data.push_back(iecStatus.connected ? 1 : 0);
        data.push_back(static_cast<uint8_t>(iecStatus.channel & 0xFF)); // it's only an int because of atoi from some basic commands, but it's never really more than 1 byte

        // max of 41 chars in message including the null terminator. It will simply be truncated, so if we find any that are excessive, should trim them down in firmware
        size_t actualLength = std::min(iecStatus.msg.length(), static_cast<size_t>(40));
        for (size_t i = 0; i < actualLength; ++i) {
            data.push_back(static_cast<uint8_t>(iecStatus.msg[i]));
        }
        data.push_back(0); // null terminate the string

        return data;
    }
};

/**
 * @class systemBus
 * @brief the system bus that all virtualDevices attach to.
 */
class systemBus
{
private:
    IECLowLevel busController;
    uint8_t iec_debug_id;

    /**
     * @brief The chain of devices on the bus.
     */
    std::forward_list<virtualDevice *> _daisyChain;

    /**
     * @brief Number of devices on bus
     */
    int _num_devices = 0;

    /**
     * @brief Enabled device bits
     */
    uint32_t enabledDevices;

    /**
     * @brief the active device being process()'ed
     */
    virtualDevice *_activeDev = nullptr;

    /**
     * @brief is device shutting down?
     */
    bool shuttingDown = false;

    void sendServiceRequest(virtualDevice *devicep, IECRecord *record);

public:
    /**
     * @brief called in main.cpp to set up the bus.
     */
    void setup();

    /**
     * @brief called from main shutdown to clean up the device.
     */
    void shutdown();

    /**
     * @brief Run one iteration of the bus service loop
     */
    void service();

    /**
     * @brief Set 2bit fast loader pair timing
     * @param set Send 's', Receive 'r'
     * @param p1 Pair 1
     * @param p2 Pair 2
     * @param p3 Pair 3
     * @param p4 Pair 4
     */
    void setBitTiming(std::string set, int p1 = 0, int p2 = 0, int p3 = 0, int p4 = 0);

    size_t read(virtualDevice *pDevice, uint8_t *buf, size_t count);
    size_t write(virtualDevice *pDevice, const uint8_t *buf, size_t count);

    /**
     * @brief send single byte
     * @param c byte to send
     * @param eoi Send EOI?
     * @return number of bytes written
    */
    size_t sendByte(const char c, bool eoi = false);

    /**
     * @brief Send bytes to bus
     * @param buf buffer to send
     * @param len length of buffer
     * @param eoi Send EOI?
     * @return number of bytes written
     */
    size_t sendBytes(const char *buf, size_t len, bool eoi = true);

    /**
     * @brief Send string to bus
     * @param s std::string to send
     * @param eoi Send EOI?
     * @return number of bytes written
     */
    size_t sendBytes(std::string s, bool eoi = true);

    /**
     * @brief called in response to RESET pin being asserted.
     */
    void reset_all_our_devices();

    /**
     * @brief Return number of devices on bus.
     * @return # of devices on bus.
     */
    int numDevices() { return _num_devices; };

    /**
     * @brief Add device to bus.
     * @param pDevice Pointer to virtualDevice
     * @param device_id The ID to assign to virtualDevice
     */
    void addDevice(virtualDevice *pDevice, int device_id);

    /**
     * @brief Remove device from bus
     * @param pDevice pointer to virtualDevice
     */
    void remDevice(virtualDevice *pDevice);

    /**
     * @brief Check if device is enabled
     * @param deviceNumber The device ID to check
     */
    bool isDeviceEnabled ( const uint8_t device_id );

    /**
     * @brief Return pointer to device given ID
     * @param device_id ID of device to return.
     * @return pointer to virtualDevice
     */
    virtualDevice *deviceById(int device_id);

    /**
     * @brief Change ID of a particular virtualDevice
     * @param pDevice pointer to virtualDevice
     * @param device_id new device ID
     */
    void changeDeviceId(virtualDevice *pDevice, int device_id);

    /**
     * @brief Are we shutting down?
     * @return value of shuttingDown
     */
    bool getShuttingDown() { return shuttingDown; }

    /**
     * @brief signal to bus that we timed out.
     */
    void senderTimeout();

};
/**
 * @brief Return
 */

extern systemBus IEC;

#endif /* IEC_H */
