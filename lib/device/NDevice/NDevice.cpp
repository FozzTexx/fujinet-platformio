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
#include "IOChannel.h" // For GET_TIMESTAMP()
#include "utils.h"
#include "debug.h"

const std::unordered_map<fujiCommandID_t, NDevice::CommandEntry> NDevice::dispatch_table = {
    { CMD::NET_OPEN,             {&NDevice::open}                   },
    { CMD::NET_CLOSE,            {&NDevice::close}                  },
    { CMD::NET_READ,             {&NDevice::read}                   },
    { CMD::NET_WRITE,            {&NDevice::write}                  },
    { CMD::NET_STATUS,           {&NDevice::status}                 },

    { CMD::NET_PARSE,            {&NDevice::do_parse}               },
    { CMD::NET_QUERY,            {&NDevice::set_query}              },
    { CMD::NET_CHANNEL_MODE,     {&NDevice::set_parser}             },
    { CMD::NET_TRANSLATION,      {&NDevice::set_translation}        },
    { CMD::NET_SET_EOL,          {&NDevice::set_eol}                },
    { CMD::NET_SET_INT_RATE,     {&NDevice::set_timer_rate}         },
#ifdef UNUSED
    { CMD::NET_HSIO_INDEX,       {&NDevice::high_speed_index}       },
    { CMD::NET_GET_DSTATS_VALUE, {&NDevice::get_dstats_value}       },
#endif /* UNUSED */
    { CMD::NET_SEEK,             {&NDevice::seek}                   },
    { CMD::NET_TELL,             {&NDevice::tell}                   },

    { CMD::NET_GETCWD,           {&NDevice::get_prefix}             },
    { CMD::NET_CHDIR,            {&NDevice::set_prefix}             },
    { CMD::NET_USERNAME,         {&NDevice::set_login}              },
    { CMD::NET_PASSWORD,         {&NDevice::set_password}           },

    { CMD::NET_RENAME,           {&NDevice::fs_rename}              },
    { CMD::NET_DELETE,           {&NDevice::fs_delete}              },
    { CMD::NET_LOCK,             {&NDevice::fs_lock}                },
    { CMD::NET_UNLOCK,           {&NDevice::fs_unlock}              },
    { CMD::NET_MKDIR,            {&NDevice::fs_mkdir}               },
    { CMD::NET_RMDIR,            {&NDevice::fs_rmdir}               },

    { CMD::NET_CONTROL,          {&NDevice::tcp_control}            },
    { CMD::NET_CLOSE_CLIENT,     {&NDevice::tcp_close_client}       },

    { CMD::NET_SET_CHANNEL_MODE, {&NDevice::http_set_channel_mode}  },

    { CMD::NET_GET_REMOTE,       {&NDevice::udp_get_remote}         },
    { CMD::NET_SET_DESTINATION,  {&NDevice::udp_set_destination}    },
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
    Debug_printf("NDevice processCommand: 0x%02x\n", packet.command());
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

bool NDevice::recognizesCommand(fujiCommandID_t command)
{
    auto it = dispatch_table.find(command);
    if (it != dispatch_table.end())
        return true;
    return false;
}

// ============================= hook defaults ===============================

std::string NDevice::network_eol() const
{
    return network_eol_override.empty() ? SYSTEM_BUS.nativeEOL() : network_eol_override;
}

#ifdef UNUSED
std::string NDevice::create_devicespec(bool is_dir)
{
    uint8_t buf[256];

    SYSTEM_BUS.transaction_get(buf, sizeof(buf));

    std::string spec((char *)buf);
    return util_devicespec_fix_for_parsing(spec, prefix, is_dir, true);
}
#endif /* UNUSED */

#ifdef HAVE_LAST_ERROR
NDeviceStatus NDevice::status_local(uint8_t mode)
{
    NDeviceStatus status;

    (void)mode;
    status.conn = false;
    status.err = lastError;
    status.avail = 0;
    return status;
}
#endif /* HAVE_LAST_ERROR */

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
#ifdef HAVE_LAST_ERROR
        lastError = protocol->error;
#endif /* HAVE_LAST_ERROR */
        Debug_printf("Protocol unable to make connection. Error: %d\n", protocol->error);
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

error_is_true NDevice::read(ByteBuffer &buf, size_t len)
{
    readAck = GET_TIMESTAMP();

    if (receiveBuffer == nullptr
        || protocol == nullptr
        || read_channel(len) != FUJI_ERROR::NONE)
        RETURN_ERROR_AS_TRUE();
    buf.resize(len);
    std::copy(receiveBuffer->begin(), receiveBuffer->begin() + len, buf.begin());
    receiveBuffer->erase(0, len);
    receiveBuffer->shrink_to_fit();
    RETURN_SUCCESS_AS_FALSE();
}

void NDevice::read(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (receiveBuffer == nullptr)
    {
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::COULD_NOT_ALLOCATE_BUFFERS;
#endif /* HAVE_LAST_ERROR */
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (protocol == nullptr)
    {
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::NOT_CONNECTED;
#endif /* HAVE_LAST_ERROR */
        SYSTEM_BUS.transaction_error();
        return;
    }

    uint16_t num_bytes = packet.param(0);

    if (!num_bytes)
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    ByteBuffer buf;
    if (read(buf, num_bytes).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_send(buf);
}

error_is_true NDevice::write(const ByteBuffer &buf)
{
#ifdef DEBUG_RAW_WRITE
    Debug_printf("writing\n%s", util_hexdump(buf.data(), buf.size()).c_str());
#endif // DEBUG_RAW_WRITE

    std::string_view view(reinterpret_cast<const char*>(buf.data()), buf.size());
    *transmitBuffer += view;
    RETURN_ERROR_IF(write_channel(view.size()) != FUJI_ERROR::NONE);
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
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::NOT_CONNECTED;
#endif /* HAVE_LAST_ERROR */
        SYSTEM_BUS.transaction_error();
        return;
    }

    ByteBuffer buf(num_bytes);
    if (SYSTEM_BUS.transaction_get(buf).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (write(buf).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_success();
}

size_t NDevice::available()
{
    size_t avail = 0;

    switch (parserMode)
    {
    case PARSER::NONE:
        avail = protocol->available();
        break;
    case PARSER::JSON:
        avail = json_bytes_remaining;
        break;
    case PARSER::SGML:
        avail = sgml_bytes_remaining;
        break;
    }

    //Debug_printf("NDevice::available=%d\n", avail);
    return avail;
}

NDeviceStatus NDevice::current_status(uint8_t mode)
{
    NDeviceStatus nstatus;

    if (protocol == nullptr)
    {
#ifdef HAVE_LAST_ERROR
        return status_local(mode);
#else
        nstatus.avail = 0;
        nstatus.conn = 0;
        nstatus.err = NDEV_STATUS::NOT_CONNECTED;
        return nstatus;
#endif /* HAVE_LAST_ERROR */
    }

    NetworkStatus ns;
    size_t avail = available();

    switch (parserMode)
    {
    case PARSER::NONE:
        protocol->status(&ns);
        break;
    case PARSER::JSON:
        ns.connected = json_bytes_remaining > 0;
        ns.error = json_bytes_remaining > 0 ? NDEV_STATUS::SUCCESS : NDEV_STATUS::END_OF_FILE;
        break;
    case PARSER::SGML:
        ns.connected = sgml_bytes_remaining > 0;
        ns.error = sgml_bytes_remaining > 0 ? NDEV_STATUS::SUCCESS : NDEV_STATUS::END_OF_FILE;
        break;
    }

    avail = std::min<size_t>(avail, 65535);

    nstatus.avail = avail;
    nstatus.conn = ns.connected;
    nstatus.err = ns.error;
#if 0
    Debug_printf("NDevice::status avail=%d conn=%d err=%d\n",
                 nstatus.avail, nstatus.conn, nstatus.err);
#endif
    return nstatus;
}

NDeviceStatus NDevice::status(uint8_t mode)
{
    readAck = GET_TIMESTAMP();
    return current_status(mode);
}

void NDevice::status(const FUJI_COMMAND_PACKET &packet)
{
    uint8_t mode = packet.param(1);

    auto nstatus = status(mode);
    readAck = GET_TIMESTAMP();
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send(&nstatus, sizeof(nstatus), false);
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

void NDevice::set_query(const std::string &query, uint8_t parseFlags)
{
    std::string buffer;

    switch (parserMode)
    {
    case PARSER::JSON:
        json->setReadQuery(query, parseFlags);
        json_bytes_remaining = json->available();
        buffer.resize(json_bytes_remaining);
        json->readValue(reinterpret_cast<uint8_t *>(buffer.data()), json_bytes_remaining);
        break;
    case PARSER::SGML:
        sgml->setReadQuery(query, parseFlags);
        sgml_bytes_remaining = sgml->available();
        buffer.resize(sgml_bytes_remaining);
        sgml->readValue(reinterpret_cast<uint8_t *>(buffer.data()), sgml_bytes_remaining);
        break;

    default:
        return;
    }

    // don't copy past first nul char in tmp
    buffer.resize(strlen(buffer.c_str()));
    *receiveBuffer += buffer;
    Debug_printf("Query set to >%s<\r\n", query.c_str());
}

void NDevice::set_query(const FUJI_COMMAND_PACKET &packet)
{
    uint8_t query_param = packet.param(1);

    std::string in(256, 0);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);
    SYSTEM_BUS.transaction_get(in);

    set_query(in, query_param);
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
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::INVALID_DEVICESPEC;
#endif /* HAVE_LAST_ERROR */
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
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::GENERAL;
#endif /* HAVE_LAST_ERROR */
        return false;
    }

    protocol->native_eol = network_eol();

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

void NDevice::set_parser(const FUJI_COMMAND_PACKET &packet)
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

void NDevice::do_parse(const FUJI_COMMAND_PACKET &packet)
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

    network_eol_override.clear();
    if (eol0 != 0x00)
    {
        network_eol_override.push_back((char)eol0);
        if (eol1 != 0x00)
            network_eol_override.push_back((char)eol1);
    }

    if (protocol != nullptr)
        protocol->native_eol = network_eol();

    SYSTEM_BUS.transaction_success();
}

void NDevice::seek(const FUJI_COMMAND_PACKET &packet)
{

    if (protocol == nullptr)
    {
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::NOT_CONNECTED;
#endif /* HAVE_LAST_ERROR */
        SYSTEM_BUS.transaction_error();
        return;
    }

    if (parserMode != PARSER::NONE)
    {
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::INVALID_POINT;
#endif /* HAVE_LAST_ERROR */
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_accept(TRANS_STATE::WILL_GET);

    u24le_t offset;
    SYSTEM_BUS.transaction_get(&offset, sizeof(offset));

    if (protocol->seek(offset, SEEK_SET) == -1)
    {
#ifdef HAVE_LAST_ERROR
        lastError = NDEV_STATUS::INVALID_POINT;
#endif /* HAVE_LAST_ERROR */
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
#ifdef HAVE_LAST_ERROR
        lastError = protocol == nullptr ? NDEV_STATUS::NOT_CONNECTED : NDEV_STATUS::INVALID_POINT;
#endif /* HAVE_LAST_ERROR */
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

/**
 * Check to see if PROCEED needs to be asserted, and assert if needed
 * (continue toggling PROCEED).
 */
bool NDevice::poll_interrupt()
{
    if (!protocol)
        return false;
    uint64_t delta = GET_TIMESTAMP() - readAck;
    if (delta < 5000)
        return false;
    delta /= 1000; // micro to milli
    delta /= timerRate;
    if (delta % 2)
        return false;
    bool hasUpdate = protocol->available() > 0;
    if (!hasUpdate)
    {
        nDevStatus_t err;
#ifdef HAVE_LAST_ERROR
        err = lastError;
#else
        auto nstatus = status(0);
        if (!nstatus.conn)
            hasUpdate = true;
        err = nstatus.err;
#endif /* HAVE_LAST_ERROR */

        hasUpdate |= err != NDEV_STATUS::SUCCESS && err != NDEV_STATUS::END_OF_FILE;
    }

    return hasUpdate;
}

void NDevice::set_timer_rate(const FujiSIOPacket &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    timerRate = packet.param8(0);
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
