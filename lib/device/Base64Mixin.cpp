#include "Base64Mixin.h"
#include "base64.h"
#include "debug.h"

void Base64Mixin::encode_input(uint16_t len)
{
    transaction_begin(TRANS_STATE::WILL_GET);

    Debug_printf("Base64Mixin: enode_input\n");

    if (!len)
    {
        Debug_printf("Invalid length. Aborting");
        transaction_error();
        return;
    }

    std::vector<unsigned char> p(len);
    transaction_get(p.data(), len);
    base64.base64_buffer += std::string((const char *)p.data(), len);
    transaction_complete();
}

void Base64Mixin::encode_compute()
{
    size_t out_len;

    /* ACK before CPU work (matches sio_hash_compute); NetSIO/tight SIO timing
     * otherwise leaves the host waiting past dtimlo (Atari status 138 timeout). */
    transaction_begin(TRANS_STATE::NO_GET);

    Debug_printf("Base64Mixin: ENCODE COMPUTE\n");

    std::unique_ptr<char[]> p = Base64::encode(base64.base64_buffer.c_str(), base64.base64_buffer.size(), &out_len);
    if (!p)
    {
        Debug_printf("base64_encode compute failed\n");
        transaction_error();
        return;
    }

    base64.base64_buffer.clear();
    base64.base64_buffer = std::string(p.get(), out_len);

    Debug_printf("Resulting BASE64 encoded data is: %u bytes\n", out_len);
    transaction_complete();
}

void Base64Mixin::encode_length()
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("Base64Mixin: ENCODE LENGTH\n");

    size_t l = base64.base64_buffer.length();
    uint8_t response[4] = {
        (uint8_t)(l >>  0),
        (uint8_t)(l >>  8),
        (uint8_t)(l >>  16),
        (uint8_t)(l >>  24)
    };

    if (!l)
    {
        Debug_printf("BASE64 buffer is 0 bytes, sending error.\n");
        transaction_put(response, sizeof(response), true);
        return;
    }

    Debug_printf("base64 buffer length: %u bytes\n", l);

    transaction_put(response, sizeof(response), false);
}

void Base64Mixin::encode_output(uint16_t len)
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("Base64Mixin: ENCODE OUTPUT\n");

    if (!len)
    {
        Debug_printf("Refusing to send a zero byte buffer. Aborting\n");
        transaction_error();
        return;
    }
    else if (len > base64.base64_buffer.length())
    {
        Debug_printf("Requested %u bytes, but buffer is only %u bytes, aborting.\n", len, base64.base64_buffer.length());
        transaction_error();
        return;
    }
    else
    {
        Debug_printf("Requested %u bytes\n", len);
    }

    std::vector<unsigned char> p(len);
    std::memcpy(p.data(), base64.base64_buffer.data(), len);
    base64.base64_buffer.erase(0, len);
    base64.base64_buffer.shrink_to_fit();

    transaction_put(p.data(), len, false);
}

void Base64Mixin::decode_input(uint16_t len)
{
    transaction_begin(TRANS_STATE::WILL_GET);

    Debug_printf("Base64Mixin: DECODE INPUT\n");

    if (!len)
    {
        Debug_printf("Invalid length. Aborting");
        transaction_error();
        return;
    }

    std::vector<unsigned char> p(len);
    transaction_get(p.data(), len);
    base64.base64_buffer += std::string((const char *)p.data(), len);
    transaction_complete();
}

void Base64Mixin::decode_compute()
{
    size_t out_len;

    transaction_begin(TRANS_STATE::NO_GET);

    Debug_printf("Base64Mixin: DECODE COMPUTE\n");

    std::unique_ptr<unsigned char[]> p = Base64::decode(base64.base64_buffer.c_str(), base64.base64_buffer.size(), &out_len);
    if (!p)
    {
        Debug_printf("base64_encode compute failed\n");
        transaction_error();
        return;
    }

    base64.base64_buffer.clear();
    base64.base64_buffer = std::string((const char *)p.get(), out_len);

    Debug_printf("Resulting BASE64 encoded data is: %u bytes\n", out_len);
    transaction_complete();
}

void Base64Mixin::decode_length()
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("Base64Mixin: DECODE LENGTH\n");

    size_t len = base64.base64_buffer.length();
    uint8_t response[4] = {
        (uint8_t)(len >>  0),
        (uint8_t)(len >>  8),
        (uint8_t)(len >>  16),
        (uint8_t)(len >>  24)
    };

    if (!len)
    {
        Debug_printf("BASE64 buffer is 0 bytes, sending error.\n");
        transaction_put(response, sizeof(response), true);
        return;
    }

    Debug_printf("base64 buffer length: %u bytes\n", len);

    transaction_put(response, sizeof(response), false);
}

void Base64Mixin::decode_output(uint16_t len)
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("Base64Mixin: DECODE OUTPUT\n");

    if (!len)
    {
        Debug_printf("Refusing to send a zero byte buffer. Aborting\n");
        transaction_error();
        return;
    }
    else if (len > base64.base64_buffer.length())
    {
        Debug_printf("Requested %u bytes, but buffer is only %u bytes, aborting.\n", len, base64.base64_buffer.length());
        transaction_error();
        return;
    }
    else
    {
        Debug_printf("Requested %u bytes\n", len);
    }

    std::vector<unsigned char> p(len);
    memcpy(p.data(), base64.base64_buffer.data(), len);
    base64.base64_buffer.erase(0, len);
    base64.base64_buffer.shrink_to_fit();
    transaction_put(p.data(), len, false);
}

bool Base64Mixin::processCommand(PROCESS_COMMAND_TYPE command)
{
    fujiCommandID_t cmd = command.command();

    switch (cmd)
    {
    case FUJICMD_BASE64_ENCODE_INPUT:
        encode_input(command.param(0));
        break;
    case FUJICMD_BASE64_ENCODE_COMPUTE:
        encode_compute();
        break;
    case FUJICMD_BASE64_ENCODE_LENGTH:
        encode_length();
        break;
    case FUJICMD_BASE64_ENCODE_OUTPUT:
        encode_output(command.param(0));
        break;
    case FUJICMD_BASE64_DECODE_INPUT:
        decode_input(command.param(0));
        break;
    case FUJICMD_BASE64_DECODE_COMPUTE:
        decode_compute();
        break;
    case FUJICMD_BASE64_DECODE_LENGTH:
        decode_length();
        break;
    case FUJICMD_BASE64_DECODE_OUTPUT:
        decode_output(command.param(0));
        break;

    default:
        return false;
    }

    return true;
}
