#include "HashMixin.h"
#include "debug.h"

#define MODE_HEX 1

void HashMixin::hash_input(uint16_t len)
{
    transaction_begin(TRANS_STATE::WILL_GET);

    Debug_printf("HashMixin: INPUT\n");

    if (!len)
    {
        Debug_printf("Invalid length. Aborting");
        transaction_error();
        return;
    }

    std::vector<unsigned char> p(len);
    transaction_get(p.data(), len);
    hasher.add_data(p);
    transaction_complete();
}

void HashMixin::hash_compute(bool clear_data, Hash::Algorithm algo)
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("HashMixin: COMPUTE\n");
    _algorithm = algo;
    hasher.compute(_algorithm, clear_data);
    transaction_complete();
}

void HashMixin::hash_length(bool as_hex)
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("HashMixin: LENGTH\n");
    uint8_t r = hasher.hash_length(_algorithm, as_hex);
    transaction_put(&r, 1, false);
}

void HashMixin::hash_output(bool as_hex)
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("HashMixin: OUTPUT\n");

    std::vector<uint8_t> hashed_data;
    if (as_hex)
    {
        std::string hex = hasher.output_hex();
        hashed_data.insert(hashed_data.end(), hex.begin(), hex.end());
    }
    else
        hashed_data = hasher.output_binary();
    transaction_put(hashed_data.data(), hashed_data.size(), false);
}

void HashMixin::hash_clear()
{
    transaction_begin(TRANS_STATE::NO_GET);
    Debug_printf("HashMixin: CLEAR\n");
    hasher.clear();
    transaction_complete();
}

bool HashMixin::processCommand(PROCESS_COMMAND_TYPE command)
{
    fujiCommandID_t cmd = command.command();

    switch (cmd)
    {
    case FUJICMD_HASH_INPUT:
        hash_input(command.param(0));
        break;
    case FUJICMD_HASH_COMPUTE:
        hash_compute(true, Hash::to_algorithm(command.param(0)));
        break;
    case FUJICMD_HASH_COMPUTE_NO_CLEAR:
        hash_compute(false, Hash::to_algorithm(command.param(0)));
        break;
    case FUJICMD_HASH_LENGTH:
        hash_length(command.param(0) == MODE_HEX);
        break;
    case FUJICMD_HASH_OUTPUT:
        hash_output(command.param(0) == MODE_HEX);
        break;
    case FUJICMD_HASH_CLEAR:
        hash_clear();
        break;

    default:
        return false;
    }

    return true;
}
