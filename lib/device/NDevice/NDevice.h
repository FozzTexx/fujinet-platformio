#ifndef NDEVICE_H
#define NDEVICE_H

#include "bus.h"
#include "network_data.h"
#include "Protocol.h"
#include "FS.h"

#define SPECIAL_BUFFER_SIZE 256

typedef struct {
    u16ne_t avail;
    uint8_t conn;
    nDevStatus_t err;
} NDeviceStatus;
static_assert(sizeof(NDeviceStatus) == 4, "NDeviceStatus must be 4 bytes");

class NDevice : public virtualDevice
{
public:
    NDevice();
    virtual ~NDevice();

    /**
     * The one bus-facing entry point. Looks up packet.command() and calls
     * the handler. Returns true if the command was recognized (regardless of
     * whether the handler itself then signaled a bus error), false if it
     * wasn't.
     */
    bool processCommand(const FUJI_COMMAND_PACKET &packet);

    // Return true if command is one that can be handled
    bool recognizesCommand(const FUJI_COMMAND_PACKET &packet);

    /** returns true when data is available */
    bool poll_interrupt();

protected:
    /**
     * Timer Rate for interrupt timer (ms)
     */
#ifdef ESP_PLATFORM
    int timerRate = 100;
#else
    int timerRate = 20;
#endif

    /**
     * The channel mode for the currently open N: device. By default it is
     * PROTOCOL, which passes read/write/status through to the protocol
     * adapter. JSON routes them to the fnJSON parser instead.
     */
    parserMode_t parserMode = PARSER::NONE;

    std::string *receiveBuffer = nullptr;
    std::string *transmitBuffer = nullptr;
    std::string *specialBuffer = nullptr;

    /** Instance of the currently open network protocol, if any. */
    std::unique_ptr<NetworkProtocol> protocol = nullptr;

    uint64_t readAck = 0;

#ifdef HAVE_LAST_ERROR
    /**
     * The last operation's error code, remembered so a later bare STATUS
     * command (no open protocol) can report it.
     */
    nDevStatus_t lastError = NDEV_STATUS::SUCCESS;
#endif /* HAVE_LAST_ERROR */

    /** Currently set prefix (CWD) for this N: device. */
    std::string prefix;

    /** Login/password for the current protocol action. */
    std::string login;
    std::string password;

    /**
     * Client-set override for the native EOL bytes a protocol adapter
     * translates to/from (NETCMD_SET_EOL). Empty means "use this platform's
     * default."
     */
    std::string network_eol_override;

    /** fnJSON wrapper, lazily created in open(), destroyed in close(). */
    FNJSON *json = nullptr;

    /** Bytes remaining in the current JSON query result. */
    unsigned short json_bytes_remaining = 0;

    /** The fnSGML parser wrapper object (HTML/XML via CSS selector) */
    FNSGML *sgml = nullptr;

    /** Bytes remaining of current SGML query result. */
    unsigned short sgml_bytes_remaining = 0;

    // ---- hooks: override only where the hardware genuinely differs -------
    /** The line ending network_eol_override resolves to when unset. */
    std::string network_eol() const;

#ifdef OBSOLETE
    /** Pull+fix up a devicespec off the bus. SIO/RS232 override to
        also strip embedded ATASCII EOL bytes. */
    virtual std::string create_devicespec(bool is_dir);
#endif /* OBSOLETE */

    /** Column width for LONG-format directory listings, or 0 to skip
        calling protocol->setDirLongWidth() at all. */
    virtual int dir_long_width() const { return 0; }

    /** Line-ending byte the fnJSON wrapper null/line-terminates values with. */
    virtual std::string json_line_ending() const { return std::string(1, '\x0a'); }
    virtual std::string sgml_line_ending() const { return std::string(1, '\x0a'); }

#ifdef HAVE_LAST_ERROR
    /**
     * Populate a STATUS reply when no protocol is bound. Default just
     * reports lastError. SIO/RS232/DriveWire override to also serve
     * IP/netmask/gateway/DNS queries via the mode byte.
     */
    virtual NDeviceStatus status_local(uint8_t mode);
#endif /* HAVE_LAST_ERROR */

    void open(const FUJI_COMMAND_PACKET &packet);
    void close(const FUJI_COMMAND_PACKET &packet);
    void read(const FUJI_COMMAND_PACKET &packet);
    virtual void write(const FUJI_COMMAND_PACKET &packet);
    virtual void status(const FUJI_COMMAND_PACKET &packet);
    void set_prefix(const FUJI_COMMAND_PACKET &packet);
    void get_prefix(const FUJI_COMMAND_PACKET &packet);
    virtual void set_query(const FUJI_COMMAND_PACKET &packet);

    error_is_true write(const ByteBuffer &buf);
    error_is_true read(ByteBuffer &buf, size_t len);
    size_t available();
    NDeviceStatus status(uint8_t mode);
    void set_query(const std::string &query, uint8_t parseFlags);

    /**
     * Parse a devicespec into a URL and instantiate the matching protocol.
     * On success, `protocol` is set and true is returned, with url_out
     * holding the parsed URL (borrow it -- don't let it outlive protocol).
     * On failure, `protocol` is left null, lastError/the bus error have
     * already been signaled, and false is returned.
     */
    bool parse_and_instantiate_protocol(std::string &deviceSpec, bool is_dir,
                                        std::unique_ptr<PeoplesUrlParser> &url_out);

    void set_login(const FUJI_COMMAND_PACKET &packet);
    void set_password(const FUJI_COMMAND_PACKET &packet);
    void set_parser(const FUJI_COMMAND_PACKET &packet);
    void do_parse(const FUJI_COMMAND_PACKET &packet);
    void set_eol(const FUJI_COMMAND_PACKET &packet);
    void seek(const FUJI_COMMAND_PACKET &packet);
    void tell(const FUJI_COMMAND_PACKET &packet);

    fujiError_t read_channel(unsigned short num_bytes);
    fujiError_t write_channel(unsigned short num_bytes);

    // fs ops -- each is its own dispatch-table entry; fs_op() below is the shared plumbing.
    void fs_rename(const FUJI_COMMAND_PACKET &packet);
    void fs_delete(const FUJI_COMMAND_PACKET &packet);
    void fs_lock(const FUJI_COMMAND_PACKET &packet);
    void fs_unlock(const FUJI_COMMAND_PACKET &packet);
    void fs_mkdir(const FUJI_COMMAND_PACKET &packet);
    void fs_rmdir(const FUJI_COMMAND_PACKET &packet);

    // tcp ops
    void tcp_control(const FUJI_COMMAND_PACKET &packet);
    void tcp_close_client(const FUJI_COMMAND_PACKET &packet);

    // http ops
    void http_set_channel_mode(const FUJI_COMMAND_PACKET &packet);

    // udp ops
    void udp_get_remote(const FUJI_COMMAND_PACKET &packet);
    void udp_set_destination(const FUJI_COMMAND_PACKET &packet);

    void set_translation(const FUJI_COMMAND_PACKET &packet) {
        (void)packet; SYSTEM_BUS.transaction_error();
    }
    void set_timer_rate(const FUJI_COMMAND_PACKET &packet);

#ifdef UNUSED
    /** NETCMD_HSIO_INDEX. SIO-only concept. Default reports unsupported. */
    virtual void high_speed_index(const FUJI_COMMAND_PACKET &packet) { (void)packet; SYSTEM_BUS.transaction_error(); }

    /** NETCMD_GET_DSTATS_VALUE: looks up the queried command's direction straight out of dispatch_table. */
    void get_dstats_value(const FUJI_COMMAND_PACKET &packet);
#endif /* UNUSED */

private:
    using Handler = void (NDevice::*)(const FUJI_COMMAND_PACKET &);

    struct CommandEntry
    {
        Handler handler;
#ifdef UNUSED
        AtariSIODirection direction;
#endif /* UNUSED */
    };

    static const std::unordered_map<uint8_t, CommandEntry> dispatch_table;

    /**
     * Shared plumbing for the six fs ops: WILL_GET-accept, parse the
     * devicespec into a one-shot protocol, dynamic_cast to NetworkProtocolFS,
     * call `op` on it, tear the one-shot protocol back down, and signal
     * success/error. Each of fs_rename/fs_delete/... is a one-line call
     * to this with a different member-function pointer for `op`.
     */
    void fs_op(const FUJI_COMMAND_PACKET &packet, fujiError_t (NetworkProtocolFS::*op)(PeoplesUrlParser *));

    NDeviceStatus current_status(uint8_t mode);
};

#endif /* NDEVICE_H */
