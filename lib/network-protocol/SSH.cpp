/**
 * SSH protocol implementation
 */

#include "SSH.h"
#include "status_error_codes.h"

const char* username="thomc";
const char* pass="e1xb64XC46";

NetworkProtocolSSH::NetworkProtocolSSH(string *rx_buf, string *tx_buf, string *sp_buf)
    : NetworkProtocol(rx_buf, tx_buf, sp_buf)
{
    Debug_printf("NetworkProtocolSSH::NetworkProtocolSSH(%p,%p,%p)\n", rx_buf, tx_buf, sp_buf);
    pfds = (struct pollfd *)calloc(1, sizeof(struct pollfd));
}

NetworkProtocolSSH::~NetworkProtocolSSH()
{
    Debug_printf("NetworkProtocolSSH::~NetworkProtocolSSH()\n");
    free(pfds);
}

bool NetworkProtocolSSH::open(EdUrlParser *urlParser, cmdFrame_t *cmdFrame)
{
    NetworkProtocol::open(urlParser, cmdFrame);
    int ret;

    if ((ret = libssh2_init(0)) != 0)
    {
        Debug_printf("NetworkProtocolSSH::open() - libssh2_init not successful. Value returned: %d\n",ret);
        error = NETWORK_ERROR_GENERAL;
        return true;
    }

    if (client.connect(urlParser->hostName.c_str(),atoi(urlParser->port.c_str())) == 0)
    {
        Debug_printf("NetworkProtocolSSH::open() - Could not connect to host. Aborting.\n");
        return true;
    }

    Debug_printf("NetworkProtocolSSH::open() - Opening session.\n");
    session = libssh2_session_init();
    if (session == nullptr)
    {
        Debug_printf("Could not create session. aborting.\n");
        return true;
    }

    Debug_printf("NetworkProtocolSSH::open() - Attempting session handshake with fd %u\n",client.fd());
    if (libssh2_session_handshake(session, client.fd()))
    {
        error = NETWORK_ERROR_GENERAL;
        Debug_printf("NetworkProtocolSSH::open() - Could not perform SSH handshake.\n");
        return true;
    }

    fingerprint = libssh2_hostkey_hash(session, LIBSSH2_HOSTKEY_HASH_SHA1);

    Debug_printf("SSH Host Key Fingerprint is: ");

    for (int i=0; i<20; i++)
    {
        Debug_printf("%02X",(unsigned char)fingerprint[i]);
        if (i<19)
            Debug_printf(":");
    }

    Debug_printf("\n");

    userauthlist = libssh2_userauth_list(session, username, strlen(username));
    Debug_printf("Authentication methods: %s\n",userauthlist);

    if (libssh2_userauth_password(session, username,pass))
    {
        error = NETWORK_ERROR_GENERAL;
        Debug_printf("Could not perform userauth.\n");
        return true;
    }

    channel = libssh2_channel_open_session(session);

    if (!channel)
    {
        error = NETWORK_ERROR_GENERAL;
        Debug_printf("Could not open session channel.\n");
        return true;
    }

    if (libssh2_channel_request_pty(channel, "vanilla"))
    {
        error = NETWORK_ERROR_GENERAL;
        Debug_printf("Could not request pty\n");
        return true;
    }

    if (libssh2_channel_shell(channel))
    {
        error = NETWORK_ERROR_GENERAL;
        Debug_printf("Could not open shell on channel\n");
        return true;
    }

    // At this point, we should be able to talk to the shell.
    Debug_printf("Shell opened.\n");

    pfds[0].fd = client.fd();
    pfds[0].events = POLLIN;

    return false;
}

bool NetworkProtocolSSH::close()
{
    libssh2_session_disconnect(session, "Closed by NetworkProtocolSSH::close()");
    libssh2_session_free(session);
    libssh2_exit();
    return false;
}

bool NetworkProtocolSSH::read(unsigned short len)
{
    return NetworkProtocol::read(len);
}

bool NetworkProtocolSSH::write(unsigned short len)
{
    bool err = false;

    return err;
}

bool NetworkProtocolSSH::status(NetworkStatus *status)
{
    NetworkProtocol::status(status);

    

    return false;
}

uint8_t NetworkProtocolSSH::special_inquiry(uint8_t cmd)
{
    return 0xFF; // selected command not implemented.
}

bool NetworkProtocolSSH::special_00(cmdFrame_t *cmdFrame)
{
    return false;
}

bool NetworkProtocolSSH::special_40(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame)
{
    return false;
}

bool NetworkProtocolSSH::special_80(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame)
{
    return false;
}