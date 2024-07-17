#ifdef BUILD_IEC
/**
 * N: Firmware
 */

#include "network.h"

#include <cstring>
#include <algorithm>

#include "../../include/debug.h"
#include "../../hardware/led.h"

#include "utils.h"

#include "status_error_codes.h"
#include "TCP.h"
#include "UDP.h"
#include "Test.h"
#include "Telnet.h"
#include "TNFS.h"
#include "FTP.h"
#include "HTTP.h"
#include "SSH.h"
#include "SMB.h"

iecNetwork::iecNetwork()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        channelMode[i] = PROTOCOL;
        protocol[i] = nullptr;
        json[i] = nullptr;
        receiveBuffer[i] = new string();
        transmitBuffer[i] = new string();
        specialBuffer[i] = new string();
    }

    iecStatus.channel = CHANNEL_COMMAND;
    iecStatus.connected = 0;
    iecStatus.msg = "fujinet network device";
    iecStatus.error = NETWORK_ERROR_SUCCESS;
}

iecNetwork::~iecNetwork()
{
    for (int i = 0; i < NUM_CHANNELS; i++)
    {
        delete protocol[i];
        delete json[i];
        delete receiveBuffer[i];
        delete transmitBuffer[i];
        delete specialBuffer[i];
        protocol[i] = nullptr;
        json[i] = nullptr;
        receiveBuffer[i] = nullptr;
        transmitBuffer[i] = nullptr;
        specialBuffer[i] = nullptr;
    }
}

void iecNetwork::poll_interrupt(unsigned char c)
{
    NetworkStatus ns;
    if (protocol[c] != nullptr)
    {
        if (protocol[c]->interruptEnable == false)
            return;

        protocol[c]->fromInterrupt = true;
        protocol[c]->status(&ns);
        protocol[c]->fromInterrupt = false;

        if (ns.rxBytesWaiting > 0 || ns.connected == 0)
            IEC.assert_interrupt();
    }
}

void iecNetwork::iec_open()
{
    bool is_raw_open = false;
    uint8_t channel_aux1 = 12;
    uint8_t channel_aux2 = translationMode[commanddata.channel]; // not sure about this, you can't set this unless you send a command for the channel first, I think it relies on the array being init to 0s

    Debug_printf("DOING OPEN\r\n");
    Debug_printv("commanddata: prim:%02x, dev:%02x, 2nd:%02x, chan:%02x\r\n", commanddata.primary, commanddata.device, commanddata.secondary, commanddata.channel);

    file_not_found = false;
    deviceSpec[commanddata.channel].clear();
    deviceSpec[commanddata.channel].shrink_to_fit();

    // Prepend prefix, if available.
    if (!prefix[commanddata.channel].empty())
        deviceSpec[commanddata.channel] += prefix[commanddata.channel];

    // Check if the payload is RAW (i.e. from fujinet-lib) by the presence of "01" as the first byte, which can't happen for BASIC.
    // If it is, then the next 2 bytes are the aux1/aux2 values (mode and trans), and the rest is the actual URL.
    // This is an efficiency so we don't have to send a 2nd command to tell it what the parameters should have been. BASIC will still need to use "openparams" command, as the OPEN line doesn't have capacity for the parameters (can't use a "," as that's a valid URL character)
    if (payload[0] == 0x01) {
        Debug_printf("RAW!\r\n");
        is_raw_open = true;
        channel_aux1 = payload[1];
        channel_aux2 = payload[2];
        payload = payload.substr(3); // remove the marker bytes so the payload can continue as with BASIC setup
    }
    Debug_printv("payload: [%s]\r\n", mstr::toHex(payload).c_str());

    if (payload != "$") {
        // fix underscores as they don't translate well to UTF8
        std::replace(payload.begin(), payload.end(), static_cast<char>(0xA4), static_cast<char>(0x5F));

        // convert to utf8. this should be ascii though. UTF will crap all over non-ascii chars
        Debug_printv("CONVERTING PAYLOAD TO UTF8\r\n");
        payload = mstr::toUTF8(payload);
        Debug_printv("payload: [%s]\r\n", mstr::toHex(payload).c_str());
        deviceSpec[commanddata.channel] += payload;
    }

    channelMode[commanddata.channel] = PROTOCOL;

    switch (commanddata.channel)
    {
    case CHANNEL_LOAD:     // load
        cmdFrame.aux1 = 4; // read
        cmdFrame.aux2 = 0; // no translation
        break;
    case CHANNEL_SAVE:     // save
        cmdFrame.aux1 = 8; // write
        cmdFrame.aux2 = 0; // no translation
        break;
    default:
        cmdFrame.aux1 = channel_aux1;
        cmdFrame.aux2 = channel_aux2;
        Debug_printf("mode: %u, translation mode: %u\r\n", cmdFrame.aux1, cmdFrame.aux2);
        break;
    }

    // Shut down protocol if we are sending another open before we close.
    if (protocol[commanddata.channel] != nullptr)
    {
        protocol[commanddata.channel]->close();
        delete protocol[commanddata.channel];
        protocol[commanddata.channel] = nullptr;
    }

    urlParser[commanddata.channel] = PeoplesUrlParser::parseURL(deviceSpec[commanddata.channel]);

    // This is unbelievably stupid, but here we are.
    // for (int i = 0; i < urlParser[commanddata.channel]->query.size(); i++)
    //     if (urlParser[commanddata.channel]->query[i]==0xa4) // underscore
    //         urlParser[commanddata.channel]->query[i]=0x5F;

    // for (int i = 0; i < urlParser[commanddata.channel]->path.size(); i++)
    //     if (urlParser[commanddata.channel]->path[i]==0xa4) // underscore
    //         urlParser[commanddata.channel]->path[i]=0x5F;

    // Convert scheme to uppercase
    std::transform(urlParser[commanddata.channel]->scheme.begin(), urlParser[commanddata.channel]->scheme.end(), urlParser[commanddata.channel]->scheme.begin(), ::toupper);

    // Instantiate based on scheme
    if (urlParser[commanddata.channel]->scheme == "TCP")
    {
        protocol[commanddata.channel] = new NetworkProtocolTCP(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "UDP")
    {
        protocol[commanddata.channel] = new NetworkProtocolUDP(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "TEST")
    {
        protocol[commanddata.channel] = new NetworkProtocolTest(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "TELNET")
    {
        protocol[commanddata.channel] = new NetworkProtocolTELNET(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "TNFS")
    {
        protocol[commanddata.channel] = new NetworkProtocolTNFS(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "FTP")
    {
        protocol[commanddata.channel] = new NetworkProtocolFTP(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "HTTP" || urlParser[commanddata.channel]->scheme == "HTTPS")
    {
        protocol[commanddata.channel] = new NetworkProtocolHTTP(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "SSH")
    {
        protocol[commanddata.channel] = new NetworkProtocolSSH(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else if (urlParser[commanddata.channel]->scheme == "SMB")
    {
        protocol[commanddata.channel] = new NetworkProtocolSMB(receiveBuffer[commanddata.channel], transmitBuffer[commanddata.channel], specialBuffer[commanddata.channel]);
    }
    else
    {
        Debug_printf("Invalid protocol: %s\r\n", urlParser[commanddata.channel]->scheme.c_str());
        file_not_found = true;
        return;
    }

    if (protocol[commanddata.channel] == nullptr)
    {
        Debug_printf("iecNetwork::open_protocol() - Could not open protocol.\r\n");
        file_not_found = true;
    }

    if (!login[commanddata.channel].empty())
    {
        protocol[commanddata.channel]->login = &login[commanddata.channel];
        protocol[commanddata.channel]->password = &password[commanddata.channel];
    }

    Debug_printf("iecNetwork::open_protocol() - Protocol %s opened.\r\n", urlParser[commanddata.channel]->scheme.c_str());

    // Attempt protocol open
    if (protocol[commanddata.channel]->open(urlParser[commanddata.channel].get(), &cmdFrame) == true)
    {
        Debug_printv("Protocol unable to make connection.\r\n");
        delete protocol[commanddata.channel];
        protocol[commanddata.channel] = nullptr;
        file_not_found = true;
        return;
    }

    // assert SRQ
    IEC.assert_interrupt();

    // Associate channel mode
    json[commanddata.channel] = new FNJSON();
    json[commanddata.channel]->setProtocol(protocol[commanddata.channel]);
}

void iecNetwork::iec_close()
{
    Debug_printf("iecNetwork::close()\r\n");

    iecStatus.channel = commanddata.channel;
    iecStatus.error = NETWORK_ERROR_SUCCESS;
    iecStatus.connected = 0;
    iecStatus.msg = "closed";

    if (json[commanddata.channel] != nullptr)
    {
        delete json[commanddata.channel];
        json[commanddata.channel] = nullptr;
    }

    // If no protocol enabled, we just signal complete, and return.
    if (protocol[commanddata.channel] == nullptr)
    {
        return;
    }
    // Ask the protocol to close
    protocol[commanddata.channel]->close();

    // Delete the protocol object
    delete protocol[commanddata.channel];
    protocol[commanddata.channel] = nullptr;
    receiveBuffer[commanddata.channel]->clear();
    receiveBuffer[commanddata.channel]->shrink_to_fit();
    transmitBuffer[commanddata.channel]->clear();
    transmitBuffer[commanddata.channel]->shrink_to_fit();
    specialBuffer[commanddata.channel]->clear();
    specialBuffer[commanddata.channel]->shrink_to_fit();

    commanddata.init();
    state = DEVICE_IDLE;
    Debug_printv("device init");
}

void iecNetwork::iec_reopen_load()
{
    NetworkStatus ns;
    bool eoi = false;

    if ((protocol[commanddata.channel] == nullptr) || (receiveBuffer[commanddata.channel] == nullptr))
    {
        Debug_printv("nullptr");
        IEC.senderTimeout();
        return; // Punch out.
    }

    if (file_not_found)
    {
        Debug_printv("file not found");
        IEC.senderTimeout();
        return;
    }

    // Get status
    protocol[commanddata.channel]->status(&ns);

    if (!ns.rxBytesWaiting)
    {
        Debug_printv("What happened?\r\n");
        IEC.senderTimeout();

        iecStatus.error = NETWORK_ERROR_GENERAL_TIMEOUT;
        iecStatus.msg = "no bytes waiting";
        iecStatus.connected = ns.connected;
        iecStatus.channel = commanddata.channel;
        return;
    }

    while (!eoi)
    {
        // Truncate bytes waiting to response size
        ns.rxBytesWaiting = (ns.rxBytesWaiting > 65534) ? 65534 : ns.rxBytesWaiting;

        Debug_printf("bytes waiting: %u connected: %u error %u \r\n", ns.rxBytesWaiting, ns.connected, ns.error);

        int blockSize = ns.rxBytesWaiting;

        Debug_printf("Reading %u bytes from stream\r\n", blockSize);

        if (protocol[commanddata.channel]->read(blockSize)) // protocol adapter returned error
        {
            iecStatus.error = NETWORK_ERROR_GENERAL;
            iecStatus.msg = "read error";
            iecStatus.connected = ns.connected;
            iecStatus.channel = commanddata.channel;
            Debug_printv("Read Error");
            IEC.senderTimeout();
            return;
        }

        // Do another status
        protocol[commanddata.channel]->status(&ns);

        if ((!ns.connected) || ns.error == 136) // EOF
            eoi = true;

        IEC.sendBytes(*receiveBuffer[commanddata.channel], true);
        receiveBuffer[commanddata.channel]->erase(0, blockSize);
    }

    iecStatus.error = NETWORK_ERROR_END_OF_FILE;
    iecStatus.msg = "eof";
    iecStatus.connected = ns.connected;
    iecStatus.channel = commanddata.channel;
}

void iecNetwork::iec_reopen_save()
{
    // If protocol isn't connected, then return not connected.
    if (protocol[commanddata.channel] == nullptr)
    {
        iecStatus.error = NETWORK_ERROR_NOT_CONNECTED;
        iecStatus.channel = commanddata.channel;
        iecStatus.msg = "not connected";
        iecStatus.connected = 0;

        Debug_printf("iec_open_save() - Not connected\r\n");
        return;
    }

    Debug_printf("Receiving data from computer...\r\n");

    while (!(IEC.flags & EOI_RECVD))
    {
        int16_t b = IEC.receiveByte();

        if (b < 0)
        {
            Debug_printf("error on receive.\r\n");
            return;
        }

        transmitBuffer[commanddata.channel]->push_back(b);
    }

    Debug_printf("Received %u bytes. Transmitting.\r\n", transmitBuffer[commanddata.channel]->length());

    if (protocol[commanddata.channel]->write(transmitBuffer[commanddata.channel]->length()))
    {
        iecStatus.error = NETWORK_ERROR_GENERAL;
        iecStatus.msg = "write error";
        iecStatus.connected = 0;
        iecStatus.channel = commanddata.channel;
    }

    transmitBuffer[commanddata.channel]->clear();
    transmitBuffer[commanddata.channel]->shrink_to_fit();
}

void iecNetwork::iec_reopen_channel()
{
    Debug_printv("primary[%2X]", commanddata.primary);
    switch (commanddata.primary)
    {
    case IEC_TALK:
        iec_reopen_channel_talk();
        break;
    case IEC_LISTEN:
        iec_reopen_channel_listen();
        break;
    }
}

void iecNetwork::iec_reopen_channel_listen()
{
    Debug_printv("channel[%2X]", commanddata.channel);

    // If protocol isn't connected, then return not connected.
    if (protocol[commanddata.channel] == nullptr)
    {
        Debug_printf("iec_reopen_channel_listen() - Not connected\r\n");
        IEC.senderTimeout();
        return;
    }

    Debug_printf("Receiving data from computer...\r\n");

    while (!(IEC.flags & EOI_RECVD))
    {
        int16_t b = IEC.receiveByte();

        if (b < 0)
        {
            Debug_printf("error on receive.\r\n");
            return;
        }

        transmitBuffer[commanddata.channel]->push_back(b);
    }

    // ok, we need to convert the data to a format the rest of the world can use. e.g. remove any trailing 0x9b from ends, and convert to ascii/utf8 and fix the underscore... sigh
    fix_petscii_data(transmitBuffer[commanddata.channel]);

    Debug_printf("Received %u bytes. Transmitting.\r\n", transmitBuffer[commanddata.channel]->length());
    Debug_printv("DATA: >%s< [%s]", transmitBuffer[commanddata.channel]->c_str(), mstr::toHex(*transmitBuffer[commanddata.channel]).c_str());

    protocol[commanddata.channel]->write(transmitBuffer[commanddata.channel]->length());
    transmitBuffer[commanddata.channel]->clear();
    transmitBuffer[commanddata.channel]->shrink_to_fit();
}

void iecNetwork::iec_reopen_channel_talk()
{
    bool set_eoi = false;
    NetworkStatus ns;

    Debug_printv("channel[%2X]", commanddata.channel);

    // If protocol isn't connected, then return not connected.
    if (protocol[commanddata.channel] == nullptr)
    {
        Debug_printf("iec_reopen_channel_talk() - Not connected\r\n");
        return;
    }

    if (receiveBuffer[commanddata.channel]->empty())
    {
        protocol[commanddata.channel]->status(&ns);

        if (ns.rxBytesWaiting)
            protocol[commanddata.channel]->read(ns.rxBytesWaiting);
    }

    if (receiveBuffer[commanddata.channel]->empty())
    {
        Debug_printv("Receive Buffer Empty.");
        IEC.senderTimeout();
        return;
    }

    do
    {
        char b = receiveBuffer[commanddata.channel]->front();

        if (receiveBuffer[commanddata.channel]->empty())
        {
            //Debug_printv("Receive Buffer Empty.");
            set_eoi = true;
        }

        IEC.sendByte(b, set_eoi);

        if ( IEC.flags & ERROR )
        {
            Debug_printv("TALK ERROR! flags[%d]\n", IEC.flags);
            return;
        }

        if ( !(IEC.flags & ATN_PULLED) )
            receiveBuffer[commanddata.channel]->erase(0, 1);

    } while( !(IEC.flags & ATN_PULLED) && !set_eoi );
}

void iecNetwork::set_login_password()
{
    int channel = 0;

    if (pt.size() == 1)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "usage login,chan,username,password";
        iecStatus.connected = 0;
        iecStatus.channel = commanddata.channel;
        return;
    }
    else if (pt.size() == 2)
    {
        // Clear login for channel X
        char reply[40];

        channel = atoi(pt[1].c_str());

        login[channel].clear();
        login[channel].shrink_to_fit();

        snprintf(reply, 40, "login cleared for channel %u", channel);

        iecStatus.error = NETWORK_ERROR_SUCCESS;
        iecStatus.msg = string(reply);
        iecStatus.connected = 0;
        iecStatus.channel = commanddata.channel;
    }
    else
    {
        char reply[40];

        channel = atoi(pt[1].c_str());

        login[channel] = pt[2];
        password[channel] = pt[3];

        snprintf(reply, 40, "login set for channel %u", channel);

        iecStatus.error = NETWORK_ERROR_SUCCESS;
        iecStatus.msg = string(reply);
        iecStatus.connected = 0;
        iecStatus.channel = commanddata.channel;
    }
}

void iecNetwork::parse_json()
{
    int channel;
    NetworkStatus ns;

    if (pt.size() < 2)
    {
        Debug_printf("parse_json - no channel specified\r\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "no channel specified";
        iecStatus.channel = 0;
        iecStatus.connected = 0;
        return;
    }

    channel = atoi(pt[1].c_str());
    protocol[channel]->status(&ns);

    if (!json[channel]->parse())
    {
        Debug_printf("could not parse json\r\n");
        iecStatus.error = NETWORK_ERROR_GENERAL;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = "could not parse json";
    }
    else
    {
        Debug_printf("json parsed\r\n");
        iecStatus.error = NETWORK_ERROR_SUCCESS;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = "json parsed";
    }
}

void iecNetwork::query_json()
{
    uint8_t *tmp;
    int channel = 0;
    char reply[80];
    string s;

    Debug_printf("query_json(%s)\r\n", payload.c_str());

    if (pt.size() < 3)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "invalid # of parameters";
        iecStatus.channel = channel;
        iecStatus.connected = 0;
        Debug_printf("Invalid # of parameters to set_json_query()\r\n");
        return;
    }

    channel = atoi(pt[1].c_str());

    s = pt[2];

    Debug_printf("Channel: %u\r\n", channel);

    // SIGH. Convert petscii underscore graphic to true underscore ascii char
    std::replace(s.begin(), s.end(), static_cast<char>(0xA4), static_cast<char>(0x5F));

    json[channel]->setReadQuery(s, 0);

    if (!json[channel]->readValueLen())
    {
        iecStatus.error = NETWORK_ERROR_COULD_NOT_ALLOCATE_BUFFERS;
        iecStatus.channel = channel;
        iecStatus.connected = 0;
        iecStatus.msg = "query not found";
        return;
    }

    tmp = (uint8_t *)malloc(json[channel]->readValueLen());

    if (!tmp)
    {
        snprintf(reply, 80, "could not allocate %u bytes for json return value", json[channel]->readValueLen());
        iecStatus.error = NETWORK_ERROR_COULD_NOT_ALLOCATE_BUFFERS;
        iecStatus.channel = channel;
        iecStatus.connected = 0;
        iecStatus.msg = string(reply);
        Debug_printf("Could not allocate %u bytes for JSON return value.\r\n", json[channel]->readValueLen());
        return;
    }

    json_bytes_remaining[channel] = json[channel]->readValueLen();
    json[channel]->readValue(tmp, json_bytes_remaining[channel]);
    *receiveBuffer[channel] += string((const char *)tmp, json_bytes_remaining[channel]);

    free(tmp);
    snprintf(reply, 80, "query set to %s", s.c_str());
    iecStatus.error = NETWORK_ERROR_SUCCESS;
    iecStatus.channel = channel;
    iecStatus.connected = true;
    iecStatus.msg = string(reply);
    Debug_printf("Query set to %s\r\n", s.c_str());
}

void iecNetwork::parse_bite()
{
    int channel = 0;
    int bite_size = 79;
    NetworkStatus ns;

    if (pt.size() < 2)
    {
        Debug_printf("parse_bite - no channel specified\r\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "no channel specified";
        iecStatus.channel = 0;
        iecStatus.connected = 0;
        return;
    }
    channel = atoi(pt[1].c_str());

    if (pt.size() == 3)
    {
        bite_size = atoi(pt[2].c_str()) - 2; // index starts at 0 and shave 1 for CR
        Debug_printv("bite_size[%d]", bite_size);
    }

    protocol[channel]->status(&ns);

    // escape chars that would break up INPUT#
    //mstr::replaceAll(*receiveBuffer[channel], ",", "\",\"");
    //mstr::replaceAll(*receiveBuffer[channel], ";", "\";\"");
    //mstr::replaceAll(*receiveBuffer[channel], ":", "\":\"");
    //mstr::replaceAll(*receiveBuffer[channel], "\r", "\"\r\"");
    //mstr::replaceAll(*receiveBuffer[channel], "\"", "\"\"");
    mstr::replaceAll(*receiveBuffer[channel], "\"", "");

    // break up receiveBuffer[channel] into bites less than bite_size bytes
    std::string bites = "\"";
    bites.reserve(receiveBuffer[channel]->size() + (receiveBuffer[channel]->size() / bite_size ));

    int start = 0;
    int end = 0;
    int len = 0;
    int count = 0;
    do
    {
        start = end;

        // Set remaining length
        len = receiveBuffer[channel]->size() - start;
        if ( len > bite_size )
            len = bite_size;

        // Don't make extra bites!
        end = receiveBuffer[channel]->find('\r', start);
        if ( end == std::string::npos )
            end = start + len; // None found so set end

        // Take a bite
        Debug_printv("start[%d] end[%d] len[%d] bite_size[%d]", start, end, len, bite_size);
        std::string bite = receiveBuffer[channel]->substr(start, len);
        bites += bite;
        Debug_printv("bite[%s]", bite.c_str());

        // Add CR if there isn't one already
        if ( len == bite_size)
             bites += "\r\"";

        count++;
    } while ( end < receiveBuffer[channel]->size() );
 
    //bites += "\"";
    //Debug_printv("[%s]", bites.c_str());
    *receiveBuffer[channel] = bites;
}

void iecNetwork::set_translation_mode()
{
    if (pt.size() < 2)
    {
        Debug_printf("no channel\r\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = 0;
        iecStatus.msg = "no channel specified";
        return;
    }
    else if (pt.size() < 3)
    {
        Debug_printf("no mode\r\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = 0;
        iecStatus.msg = "no mode specified";
    }

    int channel = atoi(pt[1].c_str());

    translationMode[channel] = atoi(pt[2].c_str());

    if (protocol[channel] == nullptr)
    {
        iecStatus.error = NETWORK_ERROR_NOT_CONNECTED;
        iecStatus.connected = 0;
    }

    iecStatus.channel = channel;

    switch (translationMode[channel])
    {
    case 0:
        iecStatus.msg = "no translation";
        break;
    case 1:
        iecStatus.msg = "atascii<->ascii CR";
        break;
    case 2:
        iecStatus.msg = "atascii<->ascii LF";
        break;
    case 3:
        iecStatus.msg = "atascii<->ascii CRLF";
        break;
    case 4:
        iecStatus.msg = "petscii<->ascii";
        break;
    }

    Debug_printf("Translation mode for channel %u is now %u\r\n", channel, translationMode[channel]);
}

void iecNetwork::iec_listen_command()
{
}

void iecNetwork::iec_talk_command()
{
    NetworkStatus ns;

    if (!active_status_channel)
    {
        Debug_printf("No active status channel\n");
        IEC.senderTimeout();
        return;
    }
    else if (protocol[active_status_channel] == nullptr)
    {
        Debug_printf("No active protocol\n");
        IEC.senderTimeout();
        return;
    }

    if (channelMode[active_status_channel] == PROTOCOL) {
        protocol[active_status_channel]->status(&ns);
    } else {
        json[active_status_channel]->status(&ns);
    }

    if (is_binary_status) {
        uint8_t binaryStatus[4];

        binaryStatus[0] = ns.rxBytesWaiting & 0xFF;        // Low byte of ns.rxBytesWaiting
        binaryStatus[1] = (ns.rxBytesWaiting >> 8) & 0xFF; // High byte of ns.rxBytesWaiting

        binaryStatus[2] = ns.connected;
        binaryStatus[3] = ns.error;

        Debug_printf("Sending status binary data for active channel: %d, %s\n", active_status_channel, mstr::toHex(binaryStatus, 4).c_str());

        IEC.sendBytes((const char *)binaryStatus, sizeof(binaryStatus), true);
    } else {
        char tmp[32];
        memset(tmp, 0, sizeof(tmp));
        sprintf(tmp, "%u,%u,%u", ns.rxBytesWaiting, ns.connected, ns.error);

        Debug_printf("Sending status %s\n", tmp);

        IEC.sendBytes(tmp, strlen(tmp), true);
    }
}

void iecNetwork::iec_command()
{
    // Check pt size before proceeding to avoid a crash
    if (pt.size()==0) {
        Debug_printf("pt.size()==0!\n");
        return;
    }

    Debug_printf("pt[0]=='%s'\n", pt[0].c_str());
    if (pt[0] == "cd")
        set_prefix();
    else if (pt[0] == "chmode")
        set_channel_mode();
    else if (pt[0] == "openparams")
        set_open_params();
    else if (pt[0] == "status")
        set_status(false);
    else if (pt[0] == "statusb")
        set_status(true);
    else if (pt[0] == "id")
        set_device_id();
    else if (pt[0] == "jsonparse")
        parse_json();
    else if (pt[0] == "jq")
        query_json();
    else if (pt[0] == "biteparse")
        parse_bite();
    else if (pt[0] == "settrans")
        set_translation_mode();
    else if (pt[0] == "pwd")
        get_prefix();
    else if (pt[0] == "login")
        set_login_password();
    else if (pt[0] == "rename" || pt[0] == "ren")
        fsop(0x20);
    else if (pt[0] == "delete" || pt[0] == "del" || pt[0] == "rm")
        fsop(0x21);
    else if (pt[0] == "lock")
        fsop(0x23);
    else if (pt[0] == "unlock")
        fsop(0x24);
    else if (pt[0] == "mkdir")
        fsop(0x2A);
    else if (pt[0] == "rmdir")
        fsop(0x2B);
    else // Protocol command processing here.
    {
        if (pt.size() > 1)
        {
            uint8_t channel = atoi(pt[1].c_str());

            // This assumption is safe, because no special commands should ever
            // be done on channel 0 (or 1 for that matter.) -tschak
            if (!channel)
                return;

            if (!protocol[channel])
            {
                Debug_printv("ERROR: trying to perform command on channel without a protocol. channel = %d, payload = >%s<\r\n", channel, payload.c_str());
                return;
            }

            Debug_printv("pt[0][0]=[%2X] pt[1]=[%d] aux1[%d] aux2[%d]", pt[0][0], channel, cmdFrame.aux1, cmdFrame.aux2);

            if (protocol[channel]->special_inquiry(pt[0][0]) == 0x00)
                perform_special_00();
            else if (protocol[channel]->special_inquiry(pt[0][0]) == 0x40)
                perform_special_40();
            else if (protocol[channel]->special_inquiry(pt[0][0]) == 0x80)
                perform_special_80();
        }
    }
}

void iecNetwork::perform_special_00()
{
    int channel = 0;

    if (pt.size() > 0)
        cmdFrame.comnd = pt[0][0];

    if (pt.size() > 1)
        channel = atoi(pt[1].c_str());

    if (pt.size() > 2)
        cmdFrame.aux1 = atoi(pt[2].c_str());

    if (pt.size() > 3)
        cmdFrame.aux2 = atoi(pt[3].c_str());

    if (protocol[channel]->special_00(&cmdFrame))
    {
        NetworkStatus ns;
        char reply[80];
        string s;

        protocol[channel]->status(&ns);
        snprintf(reply, 80, "protocol error #%u", ns.error);
        iecStatus.error = ns.error;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = ns.connected;
        s = string(reply);
        iecStatus.msg = s; // mstr::toPETSCII2(s);
    }
}

void iecNetwork::perform_special_40()
{
    char sp_buf[256];
    int channel = 0;
    NetworkStatus ns;

    if (pt.size() < 2)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = 15;
        iecStatus.connected = 0;
        iecStatus.msg = "no channel #";
        return;
    }

    channel = atoi(pt[1].c_str());
    cmdFrame.comnd = pt[0][0];

    if (pt.size() < 3)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = channel;
        iecStatus.connected = 0;
        iecStatus.msg = "no aux1";
    }

    cmdFrame.aux1 = atoi(pt[2].c_str());

    if (pt.size() < 4)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = channel;
        iecStatus.connected = 0;
        iecStatus.msg = "no aux2";
    }

    cmdFrame.aux2 = atoi(pt[3].c_str());

    if (protocol[channel] != nullptr)
    {
        iecStatus.error = NETWORK_ERROR_NOT_CONNECTED;
        iecStatus.channel = 15;
        iecStatus.connected = 0;
        iecStatus.msg = "no active protocol";
        return;
    }

    if (protocol[channel]->special_40((uint8_t *)&sp_buf, sizeof(sp_buf), &cmdFrame))
    {
        protocol[channel]->status(&ns);
        iecStatus.error = ns.error;
        iecStatus.connected = ns.connected;
        iecStatus.channel = channel;
        iecStatus.msg = "protocol read error";
        return;
    }
    else
    {
        protocol[channel]->status(&ns);
        iecStatus.error = ns.error;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = string(sp_buf);
    }
}

void iecNetwork::perform_special_80()
{
    string sp_buf = "N:";
    int channel = 0;
    NetworkStatus ns;

    Debug_printf("perform_special_80()\n");

    if (pt.size() < 2)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = 15;
        iecStatus.connected = 0;
        iecStatus.msg = "no channel #";
        return;
    }

    channel = atoi(pt[1].c_str());

    if (pt.size() < 3)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = 15;
        iecStatus.connected = 0;
        iecStatus.msg = "no aux1";
        return;
    }

    cmdFrame.aux1 = atoi(pt[2].c_str());

    if (pt.size() < 4)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = 15;
        iecStatus.connected = 0;
        iecStatus.msg = "no aux2";
        return;
    }

    cmdFrame.aux2 = atoi(pt[3].c_str());

    if (pt.size() < 5)
    {
        if (protocol[channel] != nullptr)
        {
            protocol[channel]->status(&ns);
            iecStatus.error = ns.error;
            iecStatus.connected = ns.connected;
        }
        else
        {
            iecStatus.error = NETWORK_ERROR_NOT_CONNECTED;
            iecStatus.connected = 0;
        }

        iecStatus.channel = channel;
        iecStatus.msg = "parameter missing";
    }

    cmdFrame.comnd = pt[0][0];
    sp_buf += pt[4];

    if (protocol[channel]->special_80((uint8_t *)sp_buf.c_str(), sp_buf.length(), &cmdFrame))
    {
        protocol[channel]->status(&ns);
        iecStatus.error = ns.error;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = "error";
    }
    else
    {
        protocol[channel]->status(&ns);
        iecStatus.error = ns.error;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = "ok";
    }
}

void iecNetwork::set_channel_mode()
{
    NetworkStatus ns;

    if (pt.size() < 3)
    {
        Debug_printf("set_channel_mode no channel or mode specified");
        iecStatus.error = ns.error;
        iecStatus.msg = "no channel or mode specified";
        iecStatus.connected = ns.connected;
        iecStatus.channel = commanddata.channel;
        return;
    }
    else if (pt.size() < 3)
    {
        Debug_printf("set_channel_mode no mode specified for channel %u\r\n", atoi(pt[1].c_str()));
        iecStatus.error = ns.error;
        iecStatus.msg = "no mode specified for channel " + pt[1];
        iecStatus.connected = ns.connected;
        iecStatus.channel = commanddata.channel;
    }
    else
    {
        int channel = atoi(pt[1].c_str());
        string newMode = pt[2];

        if (newMode == "json")
            channelMode[channel] = JSON;
        else if (newMode == "protocol")
            channelMode[channel] = PROTOCOL;

        Debug_printf("Channel mode set to %s %u\r\n", newMode.c_str(), channelMode[channel]);
        iecStatus.error = ns.error;
        iecStatus.channel = channel;
        iecStatus.connected = ns.connected;
        iecStatus.msg = "channel mode set to " + newMode;
    }
}

void iecNetwork::get_prefix()
{
    int channel = -1;

    if (pt.size() < 2)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.connected = 0;
        iecStatus.channel = channel;
        iecStatus.msg = "need channel #";
        return;
    }

    channel = atoi(pt[1].c_str());

    iecStatus.error = NETWORK_ERROR_SUCCESS;
    iecStatus.msg = prefix[channel];
    iecStatus.connected = 0;
    iecStatus.channel = channel;
}

void iecNetwork::set_status(bool is_binary)
{
    is_binary_status = is_binary;
    if (pt.size() < 2)
    {
        Debug_printf("Channel # Required\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "channel # required.\n";
        iecStatus.connected = 0;
        iecStatus.channel = 15;
        return;
    }

    active_status_channel = atoi(pt[1].c_str());

    Debug_printf("Active status channel now: %u\n", active_status_channel);
    iecStatus.error = NETWORK_ERROR_SUCCESS;
    iecStatus.msg = "Active status channel set.";

    if (protocol[commanddata.channel] == nullptr)
    {
        Debug_printv("protocol for channel %d is null, setting iecStatus.connected to 0\r\n", commanddata.channel);
        iecStatus.connected = 0;
    }
    else
    {

        NetworkStatus ns;
        protocol[commanddata.channel]->status(&ns);
        iecStatus.connected = ns.connected;
        Debug_printv("protocol for channel %d is set, got status bw: %d, conn: %d, err: %d:\r\n", commanddata.channel, ns.rxBytesWaiting, ns.connected, ns.error);
    }

    iecStatus.channel = atoi(pt[1].c_str());
}

void iecNetwork::set_prefix()
{
    uint8_t prefixSpec[256];
    string prefixSpec_str;
    int channel = -1;

    Debug_printf("set_prefix(%s)", payload.c_str());

    memset(prefixSpec, 0, sizeof(prefixSpec));

    if (pt.size() < 2)
    {
        Debug_printf("Channel # required\r\n");
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "channel # required";
        iecStatus.connected = 0;
        iecStatus.channel = channel;
        return;
    }
    else if (pt.size() == 2) // clear prefix
    {
        Debug_printf("Prefix cleared\r\n");
        channel = atoi(pt[1].c_str());
        prefix[channel].clear();
        prefix[channel].shrink_to_fit();
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.msg = "prefix cleared";
        iecStatus.connected = 0;
        iecStatus.channel = channel;
        return;
    }
    else
    {
        channel = atoi(pt[1].c_str());
        strncpy((char *)prefixSpec, pt[2].c_str(), 256);
    }

    util_devicespec_fix_9b(prefixSpec, sizeof(prefixSpec));

    prefixSpec_str = string((const char *)prefixSpec);
    Debug_printf("iecNetwork::set_prefix(%s)\r\n", prefixSpec_str.c_str());

    if (prefixSpec_str == "..") // Devance path N:..
    {
        std::vector<int> pathLocations;
        for (int i = 0; i < prefix[channel].size(); i++)
        {
            if (prefix[channel][i] == '/')
            {
                pathLocations.push_back(i);
            }
        }

        if (prefix[channel][prefix[channel].size() - 1] == '/')
        {
            // Get rid of last path segment.
            pathLocations.pop_back();
        }

        // truncate to that location.
        prefix[channel] = prefix[channel].substr(0, pathLocations.back() + 1);
    }
    else if (prefixSpec_str == "/") // Go back to hostname.
    {
        // TNFS://foo.com/path
        size_t pos = prefix[channel].find("/");

        if (pos == string::npos)
        {
            prefix[channel].clear();
            prefix[channel].shrink_to_fit();
        }

        pos = prefix[channel].find("/", ++pos);

        if (pos == string::npos)
        {
            prefix[channel].clear();
            prefix[channel].shrink_to_fit();
        }

        pos = prefix[channel].find("/", ++pos);

        if (pos == string::npos)
            prefix[channel] += "/";

        pos = prefix[channel].find("/", ++pos);

        prefix[channel] = prefix[channel].substr(0, pos);
    }
    else if (prefixSpec_str[0] == '/') // N:/DIR
    {
        prefix[channel] = prefixSpec_str;
    }
    else if (prefixSpec_str.empty())
    {
        prefix[channel].clear();
    }
    else if (prefixSpec_str.find_first_of(":") != string::npos)
    {
        prefix[channel] = prefixSpec_str;
    }
    else // append to path.
    {
        prefix[channel] += prefixSpec_str;
    }

    prefix[channel] = util_get_canonical_path(prefix[channel]);

    iecStatus.error = NETWORK_ERROR_SUCCESS;
    iecStatus.msg = prefix[channel];
    iecStatus.connected = 0;
    iecStatus.channel = channel;

    Debug_printf("Prefix now: %s\r\n", prefix[channel].c_str());
}

void iecNetwork::set_device_id()
{
    if (pt.size() < 2)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = 0;
        iecStatus.msg = "device id required";
        return;
    }

    int new_id = atoi(pt[1].c_str());

    IEC.changeDeviceId(this, new_id);

    iecStatus.error = 0;
    iecStatus.msg = "ok";
    iecStatus.connected = 0;
    iecStatus.channel = commanddata.channel;
}

void iecNetwork::fsop(unsigned char comnd)
{
    Debug_printf("fsop(%u)\r\n", comnd);

    if (pt.size() < 2)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = 0;
        iecStatus.msg = "invalid # of parameters";
        return;
    }

    // Overwrite payload, we no longer need command
    payload = pt[1];

    iec_open();

    cmdFrame.comnd = comnd;

    if (protocol[commanddata.channel] != nullptr)
        protocol[commanddata.channel]->perform_idempotent_80(urlParser[commanddata.channel].get(), &cmdFrame);

    iec_close();
}

void iecNetwork::set_open_params()
{
    // openparams,<channel>,<mode>,<trans>
    if (pt.size() < 3)
    {
        iecStatus.error = NETWORK_ERROR_INVALID_DEVICESPEC;
        iecStatus.channel = commanddata.channel;
        iecStatus.connected = 0;
        iecStatus.msg = "invalid # of parameters";
        return;
    }
    
    int channel = atoi(pt[1].c_str());
    int mode = atoi(pt[2].c_str());
    int trans = atoi(pt[3].c_str());

    protocol[channel]->set_open_params(mode, trans);

    iecStatus.error = 0;
    iecStatus.msg = "ok";
    iecStatus.connected = 0;
    iecStatus.channel = channel;

}

device_state_t iecNetwork::process()
{
    // Call base class
    virtualDevice::process();
    //payload=mstr::toUTF8(payload); // @idolpx? What should I do instead?

    mstr::rtrim(payload);
    // Debug_printv("payload[%s]", payload.c_str());
    // std::string hex = mstr::toHex(payload);
    // Debug_printv("hex[%s]", hex.c_str());

    Debug_printf("DOING PROCESS\r\n");
    Debug_printv("commanddata: prim:%02x, dev:%02x, 2nd:%02x, chan:%02x\r\n", commanddata.primary, commanddata.device, commanddata.secondary, commanddata.channel);

    // fan out to appropriate process routine
    switch (commanddata.channel)
    {
    case CHANNEL_LOAD:
        Debug_printv("[CHANNEL_LOAD]");
        process_load();
        break;
    case CHANNEL_SAVE:
        Debug_printv("[CHANNEL_SAVE]");
        process_save();
        break;
    case CHANNEL_COMMAND:
        Debug_printv("[CHANNEL_COMMAND]");
        process_command();
        break;
    default:
        Debug_printv("[DEFAULT - PROCESS_CHANNEL]");
        process_channel();
        break;
    }

    return state;
}

void iecNetwork::process_load()
{
    Debug_printv("secondary[%2X]", commanddata.secondary);
    switch (commanddata.secondary)
    {
    case IEC_OPEN:
        Debug_printv("[IEC_OPEN (LOAD)]");
        iec_open();
        break;
    case IEC_CLOSE:
        Debug_printv("[IEC_CLOSE (LOAD)]");
        iec_close();
        break;
    case IEC_REOPEN:
        Debug_printv("[IEC_REOPEN (LOAD)]");
        iec_reopen_load();
        break;
    default:
        break;
    }
}

void iecNetwork::process_save()
{
    Debug_printv("secondary[%2X]", commanddata.secondary);
    switch (commanddata.secondary)
    {
    case IEC_OPEN:
        Debug_printv("[IEC_OPEN (SAVE)]");
        iec_open();
        break;
    case IEC_CLOSE:
        Debug_printv("[IEC_CLOSE (SAVE)]");
        iec_close();
        break;
    case IEC_REOPEN:
        Debug_printv("[IEC_REOPEN (SAVE)]");
        iec_reopen_save();
        break;
    default:
        break;
    }
}

void iecNetwork::process_channel()
{
    Debug_printv("secondary[%2X]", commanddata.secondary);

    // we're double processing on the IEC_LISTEN and IEC_UNLISTEN phases
    if (!(commanddata.primary == IEC_LISTEN && commanddata.secondary == IEC_OPEN)) {
    // if (commanddata.primary == IEC_UNLISTEN || (commanddata.primary == IEC_LISTEN && commanddata.secondary == IEC_REOPEN)) {
        switch (commanddata.secondary)
        {
        case IEC_OPEN:
            Debug_printv("[IEC_OPEN (CHANNEL)]");
            iec_open();
            break;
        case IEC_CLOSE:
            Debug_printv("[IEC_CLOSE (CHANNEL)]");
            iec_close();
            break;
        case IEC_REOPEN:
            Debug_printv("[IEC_REOPEN (CHANNEL)]");
            iec_reopen_channel();
            break;
        default:
            break;
        }
    } else {
        Debug_printv("SKIPPING process_channel prim: %02x, 2nd: %02x", commanddata.primary, commanddata.secondary);
        return;
    }
}

void iecNetwork::process_command()
{
    Debug_printv("primary[%2X]", commanddata.primary);
    switch (commanddata.primary)
    {
    case IEC_LISTEN:
        Debug_printv("[IEC_LISTEN]");
        Debug_printv("CONVERTING PAYLOAD TO UTF8\r\n");
        payload=mstr::toUTF8(payload);
        pt = util_tokenize(payload, ',');
        break;
    case IEC_TALK:
        Debug_printv("[IEC_TALK]");
        iec_talk_command();
        break;
    case IEC_UNLISTEN:
        Debug_printv("[IEC_UNLISTEN - CALLING iec_command()]");
        iec_command();
        break;
    default:
        break;
    }

}

#endif /* BUILD_IEC */