#ifdef BUILD_ATARI

#include "sioNetwork.h"
#include "fnSystem.h"

/**
 * We were passed a COPY arg from DOS 2. This is complex, because we
 * need to parse the comma, and figure out one of three states:
 *
 * (1) we were passed D1:FOO.TXT,N:FOO.TXT, the second arg is ours.
 * (2) we were passed N:FOO.TXT,D1:FOO.TXT, the first arg is ours.
 * (3) we were passed N1:FOO.TXT,N2:FOO.TXT, get whichever one corresponds to our device ID.
 *
 * DeviceSpec will be transformed to only contain the relevant part of
 * the deviceSpec, sans comma.
 */
void sioNetwork::processCommaFromDevicespec(fujiDeviceID_t device)
{
    size_t comma_pos = deviceSpec.find(",");
    vector<string> tokens;

#error "This needs to be connected to something"

    if (comma_pos == string::npos)
        return; // no comma

    tokens = util_tokenize(deviceSpec, ',');

    for (vector<string>::iterator it = tokens.begin(); it != tokens.end(); ++it)
    {
        string item = *it;

        Debug_printf("processCommaFromDeviceSpec() found one.\n");

        if (item[0] != 'N')
            continue;                                       // not us.
        else if (item[1] == ':' && device != FUJI_DEVICEID::NETWORK) // N: but we aren't N1:
            continue;                                       // also not us.
        else
        {
            // This is our deviceSpec.
            deviceSpec = item;
            break;
        }
    }

    Debug_printf("Passed back deviceSpec %s\n", deviceSpec.c_str());
}

void sioNetwork::sio_process(const FujiSIOPacket &packet)
{
    // Let the base class handle standard commands
    if (NDevice::processCommand(packet))
        return;

    switch (packet.command())
    {
    case CMD::NET_GET_DSTATS_VALUE:
        sio_get_dstats_value(packet);
        break;
    case CMD::NET_HSIO_INDEX:
        sio_high_speed();
        break;
    default:
        break;
    }
}

/**
 * Get DSTATS value for a given command.
 *
 * This command allows CIO programs to query the data direction
 * (DSTATS) for any network command.  The command code to query is
 * passed in DAUX1 (aux1).
 *
 * Returns a single byte:
 * - SIO_DIRECTION::NONE    0x00 (no payload)
 * - SIO_DIRECTION::READ    0x40 (FujiNet→Atari)
 * - SIO_DIRECTION::WRITE   0x80 (Atari→FujiNet)
 * - SIO_DIRECTION::INVALID 0xFF (invalid command)
 */
void sioNetwork::sio_get_dstats_value(const FujiSIOPacket &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send((uint8_t) get_dstats_for_command(packet.command()));
}

/**
 * Get the DSTATS value for a given network command.
 * DSTATS indicates the direction of data for a command:
 * - SIO_DIRECTION::NONE    0x00: No payload
 * - SIO_DIRECTION::READ    0x40: Payload from FujiNet to Atari
 * - SIO_DIRECTION::WRITE   0x80: Payload from Atari to FujiNet
 * - SIO_DIRECTION::INVALID 0xFF: Invalid/unknown command
 *
 * @param command The network command code (typically from aux1)
 * @return The DSTATS byte value for that command
 */
AtariSIODirection sioNetwork::get_dstats_for_command(fujiCommandID_t command)
{
    switch (command)
    {
    // No payload commands (0x00)
    case CMD::NET_CLOSE:
    case CMD::NET_PARSE:
    case CMD::NET_CONTROL:
    case CMD::NET_CLOSE_CLIENT:
    case CMD::NET_CHANNEL_MODE:
    case CMD::NET_TRANSLATION:
    case CMD::NET_SET_INT_RATE:
    case CMD::NET_SET_PARAMETERS:
    case CMD::NET_SET_CHANNEL_MODE:
    case CMD::NET_GET_REMOTE:
        return SIO_DIRECTION::NONE;

    // Payload from FujiNet to Atari (0x40)
    case CMD::NET_HSIO_INDEX:
    case CMD::NET_READ:
    case CMD::NET_STATUS:
    case CMD::NET_GETCWD:
    case CMD::NET_TELL:
        return SIO_DIRECTION::READ;

    // Payload from Atari to FujiNet (0x80)
    case CMD::NET_OPEN:
    case CMD::NET_WRITE:
    case CMD::NET_CHDIR:
    case CMD::NET_QUERY:
    case CMD::NET_USERNAME:
    case CMD::NET_PASSWORD:
    case CMD::NET_RENAME:
    case CMD::NET_DELETE:
    case CMD::NET_LOCK:
    case CMD::NET_UNLOCK:
    case CMD::NET_MKDIR:
    case CMD::NET_RMDIR:
    case CMD::NET_SET_DESTINATION:
    case CMD::NET_SEEK:
    case CMD::NET_SET_EOL:
        return SIO_DIRECTION::WRITE;

    // Invalid/unknown command
    default:
        return SIO_DIRECTION::INVALID;
    }
}

void sioNetwork::fujidev_get_prefix(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    std::string buffer = prefix + SYSTEM_BUS.nativeEOL();
    buffer.resize(256);
    SYSTEM_BUS.transaction_send(buffer);
}

#endif /* BUILD_ATARI */
