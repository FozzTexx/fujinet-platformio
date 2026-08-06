/**
 * NetworkDeviceBase implementation.
 *
 * Compiled once per firmware target, same as every other .cpp in this tree --
 * add this file to whichever BUILD_xxx target you're building. It resolves
 * "network.h" the same way each xxx/network.cpp already resolves its own
 * "../network.h": via the include path for that target, which is expected to
 * put the relevant subdirectory first so NetworkPacket is defined before this
 * file needs it.
 */

#include "NDevice.h"
#include "NetworkProtocolFactory.h"
#include "fnjson.h"
#include "fnsgml.h"
#include "debug.h"

#define lastError _errorCode

const std::unordered_map<uint8_t, NDevice::CommandEntry> NDevice::dispatch_table = {
    { NETCMD_OPEN,             {&NDevice::open}                   },
    { NETCMD_CLOSE,            {&NDevice::close}                  },
    { NETCMD_READ,             {&NDevice::read}                   },
    { NETCMD_WRITE,            {&NDevice::write}                  },
    { NETCMD_STATUS,           {&NDevice::status}                 },

    { NETCMD_PARSE,            {&NDevice::json_parse}             },
    { NETCMD_QUERY,            {&NDevice::json_query}             },
    { NETCMD_CHANNEL_MODE,     {&NDevice::set_channel_mode}       },
    { NETCMD_TRANSLATION,      {&NDevice::set_translation}        },
    { NETCMD_SET_EOL,          {&NDevice::set_eol}                },
#ifdef UNUSED
    { NETCMD_SET_INT_RATE,     {&NDevice::set_timer_rate}         },
    { NETCMD_HSIO_INDEX,       {&NDevice::high_speed_index}       },
    { NETCMD_GET_DSTATS_VALUE, {&NDevice::get_dstats_value}       },
#endif /* UNUSED */
    { NETCMD_SEEK,             {&NDevice::seek}                   },
    { NETCMD_TELL,             {&NDevice::tell}                   },

    { NETCMD_GETCWD,           {&NDevice::get_prefix}             },
    { NETCMD_CHDIR,            {&NDevice::set_prefix}             },
    { NETCMD_USERNAME,         {&NDevice::set_login}              },
    { NETCMD_PASSWORD,         {&NDevice::set_password}           },

    { NETCMD_RENAME,           {&NDevice::fs_rename}              },
    { NETCMD_DELETE,           {&NDevice::fs_delete}              },
    { NETCMD_LOCK,             {&NDevice::fs_lock}                },
    { NETCMD_UNLOCK,           {&NDevice::fs_unlock}              },
    { NETCMD_MKDIR,            {&NDevice::fs_mkdir}               },
    { NETCMD_RMDIR,            {&NDevice::fs_rmdir}               },

    { NETCMD_CONTROL,          {&NDevice::tcp_control}            },
    { NETCMD_CLOSE_CLIENT,     {&NDevice::tcp_close_client}       },

    { NETCMD_SET_CHANNEL_MODE, {&NDevice::http_set_channel_mode}  },

    { NETCMD_GET_REMOTE,       {&NDevice::udp_get_remote}         },
    { NETCMD_SET_DESTINATION,  {&NDevice::udp_set_destination}    },
};

NDevice::NDevice()
{
    receiveBuffer = new std::string();
    transmitBuffer = new std::string();
    specialBuffer = new std::string();

    receiveBuffer->clear();
    transmitBuffer->clear();
    specialBuffer->clear();
}

NDevice::~NDevice()
{
    // protocol is a unique_ptr -- destroyed automatically.

    if (json != nullptr)
        delete json;
    json = nullptr;

    receiveBuffer->clear();
    transmitBuffer->clear();
    specialBuffer->clear();

    delete receiveBuffer;
    delete transmitBuffer;
    delete specialBuffer;
    receiveBuffer = nullptr;
    transmitBuffer = nullptr;
    specialBuffer = nullptr;
}

bool NDevice::processCommand(const FUJI_COMMAND_PACKET &packet)
{
    auto it = dispatch_table.find(packet.command());
    if (it == dispatch_table.end())
    {
        Debug_printf("NDevice::process() - unknown command: %02X\n", packet.command());
        SYSTEM_BUS.transaction_error();
        return false;
    }

    (this->*(it->second.handler))(packet);
    return true;
}

// ============================= hook defaults ===============================

#ifdef UNUSED
std::string NDevice::native_eol() const
{
    return native_eol_override.empty() ? platform_default_eol() : native_eol_override;
}
#endif /* UNUSED */

#ifdef UNUSED
std::string NDevice::create_devicespec(bool is_dir)
{
    uint8_t buf[256];

    SYSTEM_BUS.transaction_get(buf, sizeof(buf));

    std::string spec((char *)buf);
    return util_devicespec_fix_for_parsing(spec, prefix, is_dir, true);
}
#endif /* UNUSED */

void NDevice::status_local(uint8_t mode, NDeviceStatus &out)
{
    (void)mode;
    out.conn = false;
    out.err = lastError;
    out.avail = 0;
}

// ============================ shared operations =============================

void NDevice::open(const FUJI_COMMAND_PACKET &packet)
{
    fileAccessMode_t access = static_cast<fileAccessMode_t>
        (static_cast<uint8_t>(packet.param(0)));
    netProtoTranslation_t trans_mode = static_cast<netProtoTranslation_t>
        (static_cast<uint8_t>(packet.param(1)));

    std::string spec(256, 0);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    if (SYSTEM_BUS.transaction_get(spec).is_error())
    {
        Debug_printf("Failed to get device spec.");
        SYSTEM_BUS.transaction_error();
        return;
    }
    spec.resize(strlen(spec.c_str()));

    parserMode = PARSER::NONE;

    // Shut down protocol if we are sending another open before we close.
    if (protocol != nullptr)
        protocol->close();
    protocol = nullptr;

    if (json != nullptr)
    {
        delete json;
        json = nullptr;
    }

    bool is_dir = access == ACCESS_MODE::DIRECTORY;

    std::unique_ptr<PeoplesUrlParser> url;
    if (!parse_and_instantiate_protocol(spec, is_dir, url))
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (dir_long_width() > 0)
        protocol->setDirLongWidth(dir_long_width());

    if (protocol->open(url.get(), access, trans_mode) != FUJI_ERROR::NONE)
    {
        lastError = protocol->error;
        Debug_printf("Protocol unable to make connection. Error: %d\n", lastError);
        protocol = nullptr;
        SYSTEM_BUS.transaction_error();
        return;
    }

    json = new FNJSON();
    json->setLineEnding(json_line_ending());
    json->setProtocol(protocol.get());
    json_bytes_remaining = 0; // reset per-open so a prior session's count doesn't leak

    sgml = new FNSGML();
    json->setLineEnding(sgml_line_ending());
    sgml->setProtocol(protocol.get());
    sgml_bytes_remaining = 0; // reset per-open so a prior session's count doesn't leak

    parserMode = PARSER::NONE;

    SYSTEM_BUS.transaction_success();
}

void NDevice::close(const FUJI_COMMAND_PACKET &packet)
{
    (void)packet;

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (protocol == nullptr)
    {
        SYSTEM_BUS.transaction_success();
        return;
    }

    if (protocol->close() != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();

    protocol = nullptr;

    if (json != nullptr)
    {
        delete json;
        json = nullptr;
    }
}

void NDevice::read(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    readAck = GET_TIMESTAMP();

    if (receiveBuffer == nullptr)
    {
        lastError = NDEV_STATUS::COULD_NOT_ALLOCATE_BUFFERS;
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (protocol == nullptr)
    {
        lastError = NDEV_STATUS::NOT_CONNECTED;
        SYSTEM_BUS.transaction_error();
        return;
    }

    uint16_t num_bytes = packet.param(0);

    if (!num_bytes)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (read_channel(num_bytes) != FUJI_ERROR::NONE)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_send((uint8_t *)receiveBuffer->data(), num_bytes, false);
    receiveBuffer->erase(0, num_bytes);
    receiveBuffer->shrink_to_fit();
}

void NDevice::write(const FUJI_COMMAND_PACKET &packet)
{
    uint16_t num_bytes = packet.param(0);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);

    if (!num_bytes)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    if ((protocol == nullptr) || (transmitBuffer == nullptr))
    {
        lastError = NDEV_STATUS::NOT_CONNECTED;
        SYSTEM_BUS.transaction_error();
        return;
    }

    std::vector<uint8_t> buf(num_bytes);
    SYSTEM_BUS.transaction_get(buf.data(), num_bytes);

    *transmitBuffer += std::string((char *)buf.data(), num_bytes);

    if (write_channel(num_bytes) == FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_success();
    else
        SYSTEM_BUS.transaction_error();
}

void NDevice::status(const FUJI_COMMAND_PACKET &packet)
{
    uint8_t mode = packet.param(1);

    if (protocol == nullptr)
    {
        NDeviceStatus nstatus{};
        status_local(mode, nstatus);
        SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
        SYSTEM_BUS.transaction_send((uint8_t *)&nstatus, sizeof(nstatus), false);
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    NetworkStatus ns;
    size_t avail = 0;

    switch (parserMode)
    {
    case PARSER::NONE:
        protocol->status(&ns);
        avail = protocol->available();
        break;
    case PARSER::JSON:
        ns.connected = json_bytes_remaining > 0;
        ns.error = json_bytes_remaining > 0 ? NDEV_STATUS::SUCCESS : NDEV_STATUS::END_OF_FILE;
        avail = json_bytes_remaining;
        break;
    case PARSER::SGML:
        ns.connected = sgml_bytes_remaining > 0;
        ns.error = sgml_bytes_remaining > 0 ? NDEV_STATUS::SUCCESS : NDEV_STATUS::END_OF_FILE;
        avail = sgml_bytes_remaining;
        break;
    }

    avail = std::min<size_t>(avail, 65535);

    NDeviceStatus nstatus{};
    nstatus.avail = avail;
    nstatus.conn = ns.connected;
    nstatus.err = ns.error;

    SYSTEM_BUS.transaction_send((uint8_t *)&nstatus, sizeof(nstatus), false);
}

void NDevice::get_prefix(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send((uint8_t *)prefix.data(), prefix.size(), false);
}

void NDevice::set_prefix(const FUJI_COMMAND_PACKET &packet)
{
    std::string prefixSpec_str(256, 0);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(prefixSpec_str);
    Debug_printf("NDevice::set_prefix(%s)\n", prefixSpec_str.c_str());

    if (prefixSpec_str.empty()) // Nn: with nothing after it clears the prefix completely
    {
        prefix.clear();
    }
    else
    {
        if (!prefix.empty() && prefix.back() != '/')
            prefix += "/";

        // Position of the 3rd "/" in prefix (i.e. right after the host, for SCHEME://host/...).
        size_t pos = prefix.find("/");
        if (pos != std::string::npos)
        {
            pos = prefix.find("/", pos + 1);
            if (pos != std::string::npos)
                pos = prefix.find("/", pos + 1);
        }

        if (prefixSpec_str == ".." || prefixSpec_str == "<") // Devance path
        {
            prefix += "..";
        }
        else if (prefixSpec_str == "/" || prefixSpec_str == ">") // Truncate to host, e.g. TNFS://host/
        {
            if (pos != std::string::npos)
                prefix = prefix.substr(0, pos + 1);
        }
        else if (prefixSpec_str[0] == '/') // Nn:/path/to/dir/ -- keep host, replace path
        {
            if (pos != std::string::npos)
                prefix = prefix.substr(0, pos);
            prefix += prefixSpec_str;
        }
        else if (prefixSpec_str.find_first_of(":") != std::string::npos) // Nn:SCHEME://host/... -- reset entirely
        {
            prefix = prefixSpec_str;
            if (prefix.back() != '/')
                prefix += "/";
        }
        else // relative -- append to path
        {
            prefix += prefixSpec_str;
        }
    }

    prefix = util_get_canonical_path(prefix);
    Debug_printf("Prefix now: %s\n", prefix.c_str());

    SYSTEM_BUS.transaction_success();
}

void NDevice::json_query(const FUJI_COMMAND_PACKET &packet)
{
    uint8_t query_param = packet.param(1);

    std::string in(256, 0);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(in);

    json->setReadQuery(in, query_param);
    json_bytes_remaining = json->available();

    ByteBuffer tmp(json_bytes_remaining);
    json->readValue(tmp.data(), json_bytes_remaining);

    // don't copy past first nul char in tmp
    auto null_pos = std::find(tmp.begin(), tmp.end(), 0);
    *receiveBuffer += std::string(tmp.begin(), null_pos);

    Debug_printf("Query set to >%s<\r\n", in.c_str());
    SYSTEM_BUS.transaction_success();
}

bool NDevice::parse_and_instantiate_protocol(std::string &deviceSpec, bool is_dir,
                                             std::unique_ptr<PeoplesUrlParser> &url_out)
{
    deviceSpec = util_devicespec_fix_for_parsing(deviceSpec, prefix, is_dir, true);
    std::string url_str = deviceSpec.substr(deviceSpec.find(":") + 1);
    url_out = PeoplesUrlParser::parseURL(url_str);

    if (!url_out->isValidUrl())
    {
        Debug_printf("Invalid devicespec: >%s<\n", deviceSpec.c_str());
        lastError = NDEV_STATUS::INVALID_DEVICESPEC;
        protocol = nullptr;
        return false;
    }

#ifdef VERBOSE_PROTOCOL
    Debug_printf("::parse_and_instantiate_protocol -> spec: >%s<, url: >%s<\r\n", deviceSpec.c_str(), url_out->mRawUrl.c_str());
#endif

    protocol = NetworkProtocolFactory::createProtocol(url_out->scheme, receiveBuffer, transmitBuffer, specialBuffer, &login, &password);

    if (protocol == nullptr)
    {
        Debug_printf("Could not open protocol. spec: >%s<, url: >%s<\n", deviceSpec.c_str(), url_out->mRawUrl.c_str());
        lastError = NDEV_STATUS::GENERAL;
        return false;
    }

    protocol->native_eol = SYSTEM_BUS.nativeEOL();
    Debug_printf("NDevice::parse_and_instantiate_protocol() - Protocol %s created.\n", url_out->scheme.c_str());
    return true;
}

void NDevice::set_login(const FUJI_COMMAND_PACKET &packet)
{
    login.resize(256, 0);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(login);
    login.resize(strlen(login.c_str()));
    SYSTEM_BUS.transaction_success();
}

void NDevice::set_password(const FUJI_COMMAND_PACKET &packet)
{
    password.resize(256);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(password);
    password.resize(strlen(password.c_str()));
    SYSTEM_BUS.transaction_success();
}

void NDevice::set_channel_mode(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    parserMode_t mode = static_cast<parserMode_t>(static_cast<uint8_t>(packet.param(1)));
    switch (mode)
    {
    case PARSER::NONE:
    case PARSER::JSON:
    case PARSER::SGML:
        parserMode = mode;
        break;

    default:
        Debug_printf("INVALID MODE = %02x\r\n", mode);
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_success();
}

void NDevice::json_parse(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    json->parse();
    SYSTEM_BUS.transaction_success();
}

void NDevice::set_eol(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    uint8_t eol0 = packet.param(0);
    uint8_t eol1 = packet.param(1);

    native_eol_override.clear();
    if (eol0 != 0x00)
    {
        native_eol_override.push_back((char)eol0);
        if (eol1 != 0x00)
            native_eol_override.push_back((char)eol1);
    }

    if (protocol != nullptr)
        protocol->native_eol = SYSTEM_BUS.nativeEOL();

    SYSTEM_BUS.transaction_success();
}

void NDevice::seek(const FUJI_COMMAND_PACKET &packet)
{

    if (protocol == nullptr)
    {
        lastError = NDEV_STATUS::NOT_CONNECTED;
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (parserMode != PARSER::NONE)
    {
        lastError = NDEV_STATUS::INVALID_POINT;
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);

    u24le_t offset;
    SYSTEM_BUS.transaction_get(&offset, sizeof(offset));

    if (protocol->seek(offset, SEEK_SET) == -1)
    {
        lastError = NDEV_STATUS::INVALID_POINT;
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_success();
}

void NDevice::tell(const FUJI_COMMAND_PACKET &packet)
{

    uint8_t pos[3] = {0, 0, 0};
    off_t offset = -1;

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (protocol != nullptr && parserMode == PARSER::NONE)
        offset = protocol->seek(0, SEEK_CUR);

    if (offset == -1)
    {
        lastError = protocol == nullptr ? NDEV_STATUS::NOT_CONNECTED : NDEV_STATUS::INVALID_POINT;
        SYSTEM_BUS.transaction_send(pos, sizeof(pos), true);
        return;
    }

    pos[0] = offset & 0xFF;
    pos[1] = (offset >> 8) & 0xFF;
    pos[2] = (offset >> 16) & 0xFF;
    SYSTEM_BUS.transaction_send(pos, sizeof(pos), false);
}

fujiError_t NDevice::read_channel(unsigned short num_bytes)
{
    fujiError_t err = FUJI_ERROR::NONE;

    switch (parserMode)
    {
    case PARSER::NONE:
        err = protocol->read(num_bytes);
        break;
    case PARSER::JSON:
        if (num_bytes > json_bytes_remaining)
            json_bytes_remaining = 0;
        else
            json_bytes_remaining -= num_bytes;
        break;
    case PARSER::SGML:
        if (num_bytes > sgml_bytes_remaining)
            sgml_bytes_remaining = 0;
        else
            sgml_bytes_remaining -= num_bytes;
        break;
    }
    return err;
}

fujiError_t NDevice::write_channel(unsigned short num_bytes)
{
    fujiError_t err = FUJI_ERROR::NONE;

    switch (parserMode)
    {
    case PARSER::NONE:
        err = protocol->write(num_bytes);
        break;
    case PARSER::JSON:
    case PARSER::SGML:
        Debug_printf("Write not possible.\n");
        err = FUJI_ERROR::UNSPECIFIED;
        break;
    }
    return err;
}

// ================================ fs ops ====================================

void NDevice::fs_op(const FUJI_COMMAND_PACKET &packet, fujiError_t (NetworkProtocolFS::*op)(PeoplesUrlParser *))
{
    std::string spec(256, 0);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(spec);
    spec.resize(strlen(spec.c_str()));

    uint8_t mode = packet.param(0);
    bool is_dir = static_cast<fileAccessMode_t>(mode) == ACCESS_MODE::DIRECTORY;

    std::unique_ptr<PeoplesUrlParser> url;
    if (!parse_and_instantiate_protocol(spec, is_dir, url))
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    NetworkProtocolFS *fs = dynamic_cast<NetworkProtocolFS *>(protocol.get());
    if (!fs)
    {
        SYSTEM_BUS.transaction_error();
        protocol = nullptr;
        return;
    }

    fujiError_t err = (fs->*op)(url.get());

    // This was a one-shot protocol just for this fs operation.
    protocol = nullptr;

    if (err != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();
}

void NDevice::fs_rename(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::rename);
}
void NDevice::fs_delete(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::del);
}
void NDevice::fs_lock(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::lock);
}
void NDevice::fs_unlock(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::unlock);
}
void NDevice::fs_mkdir(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::mkdir);
}
void NDevice::fs_rmdir(const FUJI_COMMAND_PACKET &packet) {
    fs_op(packet, &NetworkProtocolFS::rmdir);
}

// ================================ tcp ops ===================================

void NDevice::tcp_control(const FUJI_COMMAND_PACKET &packet)
{
    NetworkProtocolTCP *tcp = dynamic_cast<NetworkProtocolTCP *>(protocol.get());
    if (!tcp)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (tcp->accept_connection() != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();
}

void NDevice::tcp_close_client(const FUJI_COMMAND_PACKET &packet)
{
    NetworkProtocolTCP *tcp = dynamic_cast<NetworkProtocolTCP *>(protocol.get());
    if (!tcp)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (tcp->close_client_connection() != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();
}

// ================================ http ops ==================================

void NDevice::http_set_channel_mode(const FUJI_COMMAND_PACKET &packet)
{
    NetworkProtocolHTTP *http = dynamic_cast<NetworkProtocolHTTP *>(protocol.get());
    if (!http)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    auto mode = (netProtoHTTPChannelMode_t)static_cast<uint8_t>(packet.param(1));
    if (http->set_channel_mode(mode) != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();
}

// ================================ udp ops ===================================

void NDevice::udp_get_remote(const FUJI_COMMAND_PACKET &packet)
{
#ifndef ESP_PLATFORM
    NetworkProtocolUDP *udp = dynamic_cast<NetworkProtocolUDP *>(protocol.get());
    if (!udp)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    fujiError_t err = udp->get_remote(receiveBuffer->data(), SPECIAL_BUFFER_SIZE);
    SYSTEM_BUS.transaction_send((uint8_t *)receiveBuffer->data(), SPECIAL_BUFFER_SIZE, err != FUJI_ERROR::NONE);
#else
    SYSTEM_BUS.transaction_error();
#endif
}

void NDevice::udp_set_destination(const FUJI_COMMAND_PACKET &packet)
{
    NetworkProtocolUDP *udp = dynamic_cast<NetworkProtocolUDP *>(protocol.get());
    if (!udp)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);

    uint8_t spData[SPECIAL_BUFFER_SIZE];
    SYSTEM_BUS.transaction_get(spData, sizeof(spData));

    if (udp->set_destination(spData, sizeof(spData)) != FUJI_ERROR::NONE)
        SYSTEM_BUS.transaction_error();
    else
        SYSTEM_BUS.transaction_success();
}

#ifdef UNUSED
// ============================ optional: DSTATS ==============================

void NDevice::get_dstats_value(const FUJI_COMMAND_PACKET &packet)
{
    uint8_t queried_command = packet.param(0);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    auto it = dispatch_table.find(queried_command);
    AtariSIODirection dir = (it == dispatch_table.end()) ? SIO_DIRECTION_INVALID : it->second.direction;

    SYSTEM_BUS.transaction_send((uint8_t)dir, false);
}
#endif /* UNUSED */
