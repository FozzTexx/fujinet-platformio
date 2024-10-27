#ifdef BUILD_IEC
#include "iec.h"
#include <string.h>
#include "../../utils/utils.h"

#include "../../include/debug.h"

systemBus IEC;

// systemBus class methods

void systemBus::remDevice(virtualDevice *pDevice)
{
    if (!pDevice)
    {
        Debug_printf("system Bus::remDevice() pDevice == nullptr! returning\r\n");
        return;
    }

    _daisyChain.remove(pDevice);
    enabledDevices &= ~ ( 1UL << pDevice->_devnum );
}

bool systemBus::isDeviceEnabled ( const uint8_t device_id )
{
    return ( enabledDevices & ( 1 << device_id ) );
} // isDeviceEnabled

void systemBus::changeDeviceId(virtualDevice *pDevice, int device_id)
{
    if (!pDevice)
    {
        Debug_printf("systemBus::changeDeviceId() pDevice == nullptr! returning.\r\n");
        return;
    }

    for (auto devicep : _daisyChain)
    {
        if (devicep == pDevice)
            devicep->_devnum = device_id;
    }
}

virtualDevice *systemBus::deviceById(int device_id)
{
    for (auto devicep : _daisyChain)
    {
        if (devicep->_devnum == device_id)
            return devicep;
    }
    return nullptr;
}

void systemBus::shutdown()
{
    shuttingDown = true;

    for (auto devicep : _daisyChain)
    {
        Debug_printf("Shutting down device #%02d\r\n", devicep->id());
        devicep->shutdown();
    }
    Debug_printf("All devices shut down.\r\n");
}

// Convert IEC commands into simple open/close/read/write
void systemBus::sendServiceRequest(virtualDevice *devicep, IECRecord *record)
{
  switch (record->command) {
  case IECListenCommand:
    switch (record->subCommand) {
    case IECOpenCommand:
      devicep->openChannel(record->channel, record->payload);
      break;

    case IECCloseCommand:
      devicep->closeChannel(record->channel);
      break;

    case IECReopenCommand:
      devicep->writeChannel(record->channel, record->payload);
      break;

    default:
      Debug_printv("Unimplemented subcommand %02x", record->subCommand);
      break;
    }
    break;

  case IECTalkCommand:
    if (record->subCommand == IECReopenCommand)
      devicep->readChannel(record->channel);
    break;

  default:
    Debug_printv("Unimplemented command %02x", record->command);
    break;
  }

  return;
}

void systemBus::setup()
{
  Debug_printv("IEC bus init %i", busController.init());
  return;
}

void systemBus::service()
{
  iec_record_header header;
  static int dlen = 0;
  static uint8_t *data = NULL;


  for (auto devicep : _daisyChain)
    if (busController.hasData(devicep->_devnum)) {
      // read data and convert to IECRecord

      read(devicep, (uint8_t *) &header, sizeof(header));

      if (dlen < header.len + 1) {
	dlen = header.len + 1;
	data = (uint8_t *) realloc(data, dlen);
      }

      if (header.len) {
	read(devicep, data, header.len);
	data[header.len] = 0;
      }

      iec_debug_id = header.serial;
      auto rec = IECRecord(header.command, header.channel, header.eoi, data, header.len);
      //rec.debugPrint();

      sendServiceRequest(devicep, &rec);
    }

  return;
}

void systemBus::addDevice(virtualDevice *pDevice, int device_id)
{
  if (!pDevice) {
    Debug_printf("systemBus::addDevice() pDevice == nullptr! returning.\r\n");
    return;
  }

  if (busController.open(device_id)) {
    Serial.printf("Failed to open device %d\r\n", device_id);
    return;
  }

  // TODO, add device shortcut pointer logic like others
  Serial.printf("Device #%02d Ready!\r\n", device_id);

  pDevice->_devnum = device_id;
  _daisyChain.push_front(pDevice);

  return;
}

void systemBus::setBitTiming(std::string set, int p1, int p2, int p3, int p4)
{
  return;
}

size_t systemBus::read(virtualDevice *pDevice, uint8_t *buf, size_t count)
{
  return busController.read(pDevice->_devnum, buf, count);
}

size_t systemBus::write(virtualDevice *pDevice, const uint8_t *buf, size_t count)
{
  return busController.write(pDevice->_devnum, buf, count);
}

size_t systemBus::sendByte(const char c, bool eoi)
{
  iec_record_header header;
  char buf[2];
  size_t wrote;


  memset(&header, 0, sizeof(header));
  header.serial = iec_debug_id;
  header.len = 1;
  header.eoi = eoi;
  buf[0] = c;
  busController.write(8, (uint8_t *) &header, sizeof(header));
  wrote = busController.write(8, (uint8_t *) buf, 1);

  return wrote;
}

// Convenience wrappers for write()

size_t systemBus::sendBytes(std::string s, bool eoi)
{
  return sendBytes(s.c_str(), s.size(), eoi);
}

size_t systemBus::sendBytes(const char *buf, size_t len, bool eoi)
{
  iec_record_header header;
  size_t wrote;


  memset(&header, 0, sizeof(header));
  header.serial = iec_debug_id;
  header.len = len;
  header.eoi = eoi;
  busController.write(8, (uint8_t *) &header, sizeof(header));
  wrote = busController.write(8, (uint8_t *) buf, header.len);

  return wrote;
}

// aka FILE NOT FOUND
void systemBus::senderTimeout()
{
  iec_record_header header;


  memset(&header, 0, sizeof(header));
  header.serial = iec_debug_id;
  busController.write(8, (uint8_t *) &header, sizeof(header));

  return;
}

// IECRecord class methods

void IECRecord::debugPrint() const
{
  Debug_printf("\r\nIECRecord cmd: %02x  sub: %02x  chan: %02x  eoi: %i  length: %i",
	       command, subCommand, channel, eoi, payload.size());
  if (payload.size())
    Debug_printf("\r\n%s", util_hexdump(payload.data(), payload.size()).c_str());
}

// IECPayload class methods

const std::string &IECPayload::string() const
{
  if (!_dataString)
    _dataString = std::string(_data.begin(), _data.end());
  return *_dataString;
}

std::vector<std::string> IECPayload::tokenize(char c) const
{
  return util_tokenize(string(), ',');
}

const uint8_t *IECPayload::data() const
{
  return _data.data();
}

size_t IECPayload::size() const
{
  return _data.size();
}

const uint8_t &IECPayload::operator[](size_t index) const
{
  return _data[index];
}

size_t IECPayload::find(const std::string &str, size_t pos) const
{
  return string().find(str, pos);
}

#endif /* BUILD_IEC */
