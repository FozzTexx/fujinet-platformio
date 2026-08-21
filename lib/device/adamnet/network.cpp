#ifdef BUILD_ADAM

#include "network.h"
#include "debug.h"

#define MAX_ADAM_PACKET_LEN 1024

AdamNetStatus adamNetwork::deviceStatus()
{
    AdamNetStatus status;
    NetworkStatus s;

    if (protocol != nullptr)
    {
        // passive: a bus status poll must not trigger deferred protocol
        // work (e.g. the lazy HTTP transaction) inside the reply deadline
        protocol->fromInterrupt = true;
        protocol->status(&s);
        protocol->fromInterrupt = false;
        statusByte.bits.client_connected = s.connected == true;
        statusByte.bits.client_data_available = protocol->available() > 0;
        statusByte.bits.client_error = s.error != NDEV_STATUS::SUCCESS;
    }

    status.length = MAX_ADAM_PACKET_LEN;
    status.devtype = ADAMNET_DEVTYPE::CHAR;
    status.status = statusByte.byte;

    return status;
}

void adamNetwork::write(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (!packet.data().has_value()
        || NDevice::write(packet.data().value()).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_success();
}

void adamNetwork::adamnet_control_receive()
{
    ByteBuffer buf;
    size_t avail = std::min<size_t>(available(), MAX_ADAM_PACKET_LEN);

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (!avail)
    {
        SYSTEM_BUS.transaction_success();
        return;
    }

    if (NDevice::read(buf, avail).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    Debug_printf("adamnetNetwork::read len=%d\n", buf.size());
    SYSTEM_BUS.transaction_send(buf);
}

void adamNetwork::status(const FUJI_COMMAND_PACKET &packet)
{
    auto nstatus = NDevice::status(0);
    Debug_printf("adamNetwork::status avail=%d conn=%d err=%d\n",
                 nstatus.avail, nstatus.conn, nstatus.err);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send(&nstatus, sizeof(nstatus), false);
}

void adamNetwork::json_query(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    NDevice::json_query(packet.dataAsString().value_or(""), 0);
    SYSTEM_BUS.transaction_success();
}

#endif /* BUILD_ADAM */
