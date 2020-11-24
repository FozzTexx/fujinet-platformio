/**
 * Test protocol implementation
 */

#include "Test.h"

NetworkProtocolTest::NetworkProtocolTest(string *rx_buf, string *tx_buf, string *sp_buf)
    : NetworkProtocol(rx_buf, tx_buf, sp_buf)
{
    Debug_printf("NetworkProtocolTest::NetworkProtocolTest(%p,%p,%p)\n",rx_buf,tx_buf,sp_buf);
}

NetworkProtocolTest::~NetworkProtocolTest()
{
    Debug_printf("NetworkProtocolTest::~NetworkProtocolTest()\n");
}

bool NetworkProtocolTest::open(EdUrlParser *urlParser, cmdFrame_t *cmdFrame)
{
    Debug_printf("scheme: %s\n",urlParser->scheme.c_str());
    Debug_printf("path: %s\n",urlParser->path.c_str());
    Debug_printf("port: %s\n",urlParser->port.c_str());
    Debug_printf("query: %s\n",urlParser->query.c_str());
    return false;
}

bool NetworkProtocolTest::close()
{
    return false;
}

bool NetworkProtocolTest::read(unsigned short len)
{
    return false;
}

bool NetworkProtocolTest::write(unsigned short len)
{
    return false;
}

bool NetworkProtocolTest::status(NetworkStatus* status)
{
    status->error=error;
    return false;
}

uint8_t NetworkProtocolTest::special_inquiry(uint8_t cmd)
{
    return 0xFF; // selected command not implemented.
}

bool NetworkProtocolTest::special_00(cmdFrame_t *cmdFrame)
{
    return false;
}

bool NetworkProtocolTest::special_40(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame)
{
    return false;
}

bool NetworkProtocolTest::special_80(uint8_t *sp_buf, unsigned short len, cmdFrame_t *cmdFrame)
{
    return false;
}