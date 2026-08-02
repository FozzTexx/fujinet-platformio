#ifdef BUILD_LYNX

/**
 * Comlynx Functions
 */
#include "comlynx.h"
#include "netstream.h"

#include "../../include/debug.h"

#include "fnSystem.h"
#include "fnDNS.h"
#include "led.h"
#include "fnConfig.h"
#include <cstring>

#ifdef ESP_PLATFORM
#define SERIAL_DEVICE FN_UART_BUS
#else /* !ESP_PLATFORM */
#define SERIAL_DEVICE Config.get_serial_port()
#endif /* ESP_PLATFORM */

//#define IDLE_TIME 500 // Idle tolerance in microseconds (roughly three characters at 62500 baud)


uint8_t comlynx_checksum(const uint8_t *buf, unsigned short len)
{
    uint8_t checksum = 0x00;

    for (unsigned short i = 0; i < len; i++)
        checksum ^= buf[i];

    return checksum;
}

#ifdef OBSOLETE
void virtualDevice::comlynx_send(uint8_t b)
{
    //Debug_printf("comlynx_send_buffer - %X\n", b);

    // Wait for idle only when in netstream mode
    if (SYSTEM_BUS.netstreamActive())
        SYSTEM_BUS.wait_for_idle();

    // Write the byte
    SYSTEM_BUS.write(b);
    SYSTEM_BUS.read();
}

void virtualDevice::comlynx_send_buffer(const uint8_t *buf, unsigned short len)
{
    Debug_printf("comlynx_send_buffer - len:%d\n", len);

    // Wait for idle only when in netstream mode
    if (SYSTEM_BUS.netstreamActive())
        SYSTEM_BUS.wait_for_idle();

    SYSTEM_BUS.write(buf, len);
}

bool virtualDevice::comlynx_recv_ck()
{
    uint8_t recv_ck, ck;


    while (SYSTEM_BUS.available() <= 0)
        fnSystem.yield();

    // get checksum
    recv_ck = SYSTEM_BUS.read();

    ck = comlynx_checksum(recvbuffer, recvbuffer_len);

    if (recv_ck == ck)
        return true;
    else
        return false;
}

uint8_t virtualDevice::comlynx_recv()
{
    uint8_t b;

    while (SYSTEM_BUS.available() <= 0)
        fnSystem.yield();

    b = SYSTEM_BUS.read();

    // Add to receive buffer
    recvbuffer[recvbuffer_len] = b;
    recvbuffer_len++;

    //Debug_printf("comlynx_recv: %x\n", b);
    return b;
}

/*bool virtualDevice::comlynx_recv_timeout(uint8_t *b, uint64_t dur)
{
    uint64_t start, current, elapsed;
    bool timeout = true;

    start = current = esp_timer_get_time();
    elapsed = 0;

    while (SYSTEM_BUS.available() <= 0)
    {
        current = esp_timer_get_time();
        elapsed = current - start;
        if (elapsed > dur)
            break;
    }

    if (SYSTEM_BUS.available() > 0)
    {
        *b = (uint8_t)SYSTEM_BUS.read();
        timeout = false;
    } // else
      //   Debug_printf("duration: %llu\n", elapsed);

    return timeout;
}*/

uint16_t virtualDevice::comlynx_recv_length()
{
    unsigned short l = 0;
    l = comlynx_recv() << 8;
    l |= comlynx_recv();

    if (l > 1024)
        l = 1024;

    // Reset recv buffer
    recvbuffer_len = 0;
    recvbuf_pos = &recvbuffer[0];

    return l;
}

void virtualDevice::comlynx_send_length(uint16_t l)
{
    comlynx_send(l >> 8);
    comlynx_send(l & 0xFF);

    #ifdef DEBUG
        Debug_printf("comlynx_send_length - len:%ld\n", (long int)l);
    #endif
}

unsigned short virtualDevice::comlynx_recv_buffer(uint8_t *buf, unsigned short len)
{
    unsigned short b;

    b = SYSTEM_BUS.read(buf, len);

    // Add to receive buffer
    memcpy(recvbuffer, buf, len);
    recvbuffer_len = len;               // length of payload
    recvbuf_pos = &recvbuffer[0];       // pointer into payload

    return(b);
}
#endif /* OBSOLETE */

void virtualDevice::reset()
{
    Debug_printf("No Reset implemented for device %u\n", _devnum);
}

#ifdef OBSOLETE
void virtualDevice::comlynx_response_ack()
{
    comlynx_send(FUJICMD_ACK);
}

void virtualDevice::comlynx_response_nack()
{
    comlynx_send(FUJICMD_NAK);
}
#endif /* OBSOLETE */

bool systemBus::wait_for_idle()
{
    int64_t start, current, dur;

    // SJ notes: we really don't need to do this unless we are in netstream mode
    // Likely we want to just wait until the bus is "idle" for about 3 character times
    // which is about 0.5 ms at 62500 baud 8N1
    //
    // Check that the bus is truly idle for the whole duration, and then we can start sending?

    start = GET_TIMESTAMP();

    do {
        current = GET_TIMESTAMP();
        dur = current - start;

        // Did we get any data in the FIFO while waiting?
        if (available() > 0)
            return false;

    } while (dur < COMLYNX_IDLE_TIME);

    // Must have been idle at least IDLE_TIME to get here
    return true;

    //fnSystem.yield();         // not sure if we need to do this, from old function - SJ
}

bool systemBus::netstreamActive() const
{
    return _streamDev != nullptr && _streamDev->netstreamActive;
}

void virtualDevice::comlynx_process(const FujiLynxPacket &packet)
{
    Debug_printf("comlynx_process() not implemented yet for this device.\n");
}

void systemBus::_comlynx_process_cmd()
{
    fujiDeviceID_t dev;
    u16be_t len;
    ByteBuffer buffer, payload;
    uint8_t ck;
    size_t rlen;

    buffer.resize(3, 0);
    rlen = read(buffer.data(), buffer.size());
    if (rlen != buffer.size())
    {
        Debug_printf("failed to read packet header\n");
        sendNakPacket();
        return;
    }

    memcpy(&len, buffer.data() + 1, sizeof(len));
    payload.resize(len, 0);
    rlen = read(payload.data(), payload.size());
    if (rlen != payload.size())
        payload.resize(rlen);
    buffer.insert(buffer.end(), payload.begin(), payload.end());
    buffer.push_back(read());

    Debug_printf("Received packet\n%s", util_hexdump(buffer.data(), buffer.size()).c_str());

    auto tmpPacket = FujiLynxPacket::fromSerialized(buffer);
    if (!tmpPacket)
    {
        Debug_printf("bad packet\n");
        sendNakPacket();
        _port->discardInput();
        goto done;
    }

    sendAckPacket();

    for (auto devicep : _daisyChain)
    {
        if (tmpPacket->device() == devicep->_devnum)
        {
            _activeDev = devicep;
            _activePacket = tmpPacket.get();

            #ifdef DEBUG
            Debug_println("---");
            Debug_printf("comlynx_process_cmd - dev:%X\n", tmpPacket->device());
            #endif

            // turn on Comlynx Indicator LED
            fnLedManager.set(eLed::LED_BUS, true);
            devicep->comlynx_process(*tmpPacket);
            // turn off Comlynx Indicator LED
            fnLedManager.set(eLed::LED_BUS, false);
        }
    }

    // Find device ID and pass control to it
    /*if (_daisyChain.count(d) < 1)
    {
    }
    else if (_daisyChain[d]->device_active == true)
    {
     #ifdef DEBUG
        Debug_println("---");
        Debug_printf("comlynx_process_cmd - dev:%X\n", d);
    #endif

        // turn on Comlynx Indicator LED
        fnLedManager.set(eLed::LED_BUS, true);
        _daisyChain[d]->comlynx_process();
        // turn off Comlynx Indicator LED
        fnLedManager.set(eLed::LED_BUS, false);
    }*/

 done:
    flush();
}

void systemBus::_comlynx_process_queue()
{
}

void systemBus::service()
{
    // Handle NetStream if active
    if (_streamDev != nullptr && _streamDev->netstreamActive) {
        if (_streamDev->redeye_mode)
            _streamDev->comlynx_handle_redeye_netstream();
        else
            _streamDev->comlynx_handle_netstream();
    }
    // Process anything waiting
    else if (available() > 0)
        _comlynx_process_cmd();
}

void systemBus::setup()
{
    Debug_println("COMLYNX SETUP");

    // Set up NetStream device
    //_streamDev = new lynxnetstream();

    if (Config.get_boip_enabled())
    {
        Debug_printf("RS232 SETUP: BOIP host: %s\n", Config.get_boip_host().c_str());
        _boip.begin(BoIPConfig()
                    .hostName(Config.get_boip_host())
                    .portNum(Config.get_boip_port())
                    );
        _port = &_boip;
    }
    else {
        // Set up UART
        _serial.begin(ChannelConfig()
                      .deviceID(SERIAL_DEVICE)
                      .baud(COMLYNX_BAUDRATE)
#ifdef ESP_PLATFORM
                      .parity(UART_PARITY_ODD)
                      .halfDuplex(true)
#endif /* ESP_PLATFORM */
                      );
        _port = &_serial;
    }
}

void systemBus::shutdown()
{
    for (auto devicep : _daisyChain)
    {
        Debug_printf("Shutting down device %02x\n", devicep->id());
        devicep->shutdown();
    }
    Debug_printf("All devices shut down.\n");
}

void systemBus::addDevice(virtualDevice *pDevice, fujiDeviceID_t device_id)
{
    Debug_printf("Adding device: %02X\n", device_id);

    if (device_id == FUJI_DEVICEID_FUJINET)
    {
        _fujiDev = (lynxFuji *)pDevice;
    }
    else if (device_id >= FUJI_DEVICEID_NETWORK && device_id <= FUJI_DEVICEID_NETWORK_LAST)
    {
        _netDev[device_id - FUJI_DEVICEID_NETWORK] = (lynxNetwork*)pDevice;
    }
    else if (device_id == FUJI_DEVICEID_PRINTER)
    {
        _printerDev = (lynxPrinter *)pDevice;
    }
    else if (device_id == FUJI_DEVICEID_MIDI)
    {
        _streamDev = (lynxNetStream *)pDevice;
    }

    pDevice->_devnum = device_id;
    _daisyChain.push_front(pDevice);
}

void systemBus::remDevice(virtualDevice *pDevice)
{
    _daisyChain.remove(pDevice);
}

void systemBus::remDevice(fujiDeviceID_t device_id)
{
}

int systemBus::numDevices()
{
      int i = 0;
    //__BEGIN_IGNORE_UNUSEDVARS
    for (auto devicep : _daisyChain)
        i++;
    return i;
    //__END_IGNORE_UNUSEDVARS
}

void systemBus::changeDeviceId(virtualDevice *p, int device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep == p)
            devicep->_devnum = (fujiDeviceID_t) device_id;
    }
}

virtualDevice *systemBus::deviceById(fujiDeviceID_t device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep->_devnum == device_id)
            return devicep;
    }
    return nullptr;
}

void systemBus::reset()
{
    for (auto devicep : _daisyChain)
        devicep->reset();
}

void systemBus::enableDevice(fujiDeviceID_t device_id)
{
}

void systemBus::disableDevice(fujiDeviceID_t device_id)
{
}

void systemBus::setStreamHost(const char *hostname, int port)
{
    setStreamHostWithOptions(hostname, port, 0, false, true);
}

void systemBus::setStreamHostWithOptions(const char *hostname, int port, int mode, bool register_enabled, bool redeye_enabled)
{
    // Turn off if hostname is STOP
    if (hostname != nullptr && !strcmp(hostname, "STOP"))
    {
        if (_streamDev->netstreamActive)
            _streamDev->comlynx_disable_netstream();

        return;
    }

    if (hostname != nullptr && hostname[0] != '\0')
    {
        // Try to resolve the hostname and store that so we don't have to keep looking it up
        _streamDev->netstream_host_ip = get_ip4_addr_by_name(hostname);
        //_streamDev->netstream_host_ip = IPADDR_NONE;

        if (_streamDev->netstream_host_ip == IPADDR_NONE)
        {
            Debug_printf("Failed to resolve hostname \"%s\"\n", hostname);
        }
    }
    else
    {
        _streamDev->netstream_host_ip = IPADDR_NONE;
    }

    if (port > 0 && port <= 65535)
    {
        _streamDev->netstream_port = port;
    }
    else
    {
        _streamDev->netstream_port = 5004;
        Debug_printf("netstream port not provided or invalid (%d), setting to 5004\n", port);
    }

    _streamDev->netstreamMode = (mode == 0)
        ? lynxNetStream::NetStreamMode::UDP
        : lynxNetStream::NetStreamMode::TCP;
    _streamDev->netstreamRegisterEnabled = register_enabled;
    _streamDev->redeye_mode = redeye_enabled;

    // Restart NetStream mode if needed
    if (_streamDev->netstreamActive) {
        _streamDev->comlynx_disable_netstream();
        _streamDev->comlynx_disable_redeye();
    }
    if (_streamDev->netstream_host_ip != IPADDR_NONE) {
        _streamDev->comlynx_enable_netstream();
        if (_streamDev->redeye_mode)
            _streamDev->comlynx_enable_redeye();
    }
}

void systemBus::setRedeyeMode(bool enable)
{
    Debug_printf("setRedeyeMode, %d\n", enable);
    _streamDev->redeye_mode = enable;
    _streamDev->redeye_reset_game();
}

void systemBus::setRedeyeGameRemap(uint32_t remap)
{
    Debug_printf("setRedeyeGameRemap, %d\n", (int) remap);

    // handle pure updstream games
    if ((remap >> 8) == 0xE1) {
        _streamDev->redeye_mode = false;           // turn off redeye
        _streamDev->game.game_id = remap;          // set game, since we can't detect it
        _streamDev->game.remap_game_id = remap;
        return;
    }

    // handle redeye game that need remapping
    _streamDev->redeye_mode = true;
    if (remap != 0xFFFF) {
        _streamDev->game.remap_game_id = remap;
    }
    else {
        _streamDev->game.remap_game_id = 0;
    }
}

void systemBus::transaction_accept(transState_t expectMoreData)
{
    assert(_transaction_state == TRANS_STATE::INVALID);
    _transaction_state = expectMoreData;
}

void systemBus::transaction_success()
{
    assert(_transaction_state == TRANS_STATE::NO_GET || _transaction_state == TRANS_STATE::DID_GET);
    Debug_println("transaction_complete - sent ACK");
    sendAckPacket();
    _transaction_state = TRANS_STATE::INVALID;
}

void systemBus::transaction_error()
{
    Debug_println("transaction_error - send NAK");
    sendNakPacket();

    // throw away any waiting bytes
    _port->discardInput();
}

success_is_true systemBus::transaction_get(void *data, size_t len)
{
    assert(_transaction_state == TRANS_STATE::WILL_GET);
    _transaction_state = TRANS_STATE::DID_GET;
    auto to_copy = std::min<size_t>(len, _activePacket->data()->size());
    std::copy(_activePacket->data()->begin(), _activePacket->data()->begin() + to_copy,
              static_cast<uint8_t *>(data));
    RETURN_SUCCESS_IF(to_copy != 0);
}

void systemBus::transaction_send(const void *data, size_t len, bool err)
{
    const uint8_t *ptr = reinterpret_cast<const uint8_t *>(data);

    assert(_transaction_state == TRANS_STATE::NO_GET);

    // send all data back to Lynx
    FujiLynxPacket packet(_activeDev->_devnum, ByteBuffer(ptr, ptr + len));
    auto encoded = packet.serialize();
    Debug_printf("Sending reply\n%s", util_hexdump(encoded.data(), encoded.size()).c_str());
    _port->write(encoded.data(), encoded.size());

    // get ACK or NACK from Lynx, we're ignoring currently
    uint8_t r = _port->read();
#ifdef DEBUG
    if (r == FUJICMD_ACK)
        Debug_println("transaction_put - Lynx ACKed");
    else
        Debug_println("transaction put - Lynx NAKed");
#endif

    _transaction_state = TRANS_STATE::INVALID;
    return;
}

void systemBus::sendAckPacket()
{
    _port->write(FUJICMD_ACK);
}

void systemBus::sendNakPacket()
{
    _port->write(FUJICMD_NAK);
}

void systemBus::change_baud(int32_t baud)
{
    _port->flushOutput();
    if (_port == &_serial)
    {
        _serial.begin(ChannelConfig()
                      .deviceID(SERIAL_DEVICE)
                      .baud(baud)
#ifdef ESP_PLATFORM
                      .parity(UART_PARITY_ODD)
#endif /* ESP_PLATFORM */
                      );
    }

#ifdef ESP_PLATFORM
    vTaskDelay(pdMS_TO_TICKS(10));
#endif /* ESP_PLATFORM */

    //uart_set_baudrate(FN_UART_BUS, baud);
    //_port->setBaudrate(baud);
}

#endif /* BUILD_LYNX */
