#ifdef BUILD_ADAM

#include "adamNetwork.h"
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

void adamNetwork::fujidev_write(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (!packet.data().has_value()
        || fujicore_write(packet.data().value()).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    SYSTEM_BUS.transaction_success();
}

void adamNetwork::adamnet_control_receive()
{
    ByteBuffer buf;
#ifdef UNUSED
    size_t avail = std::min<size_t>(fujicore_available(), MAX_ADAM_PACKET_LEN);
#else
    NDeviceStatus nstatus = fujicore_status();
    size_t avail = nstatus.avail;
#endif /* UNUSED */

    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);

    if (!avail)
    {
        SYSTEM_BUS.transaction_success();
        return;
    }

    if (fujicore_read(buf, avail).is_error())
    {
        SYSTEM_BUS.transaction_error();
        return;
    }

    Debug_printf("adamnetNetwork::read len=%d\n", buf.size());
    SYSTEM_BUS.transaction_send(buf);
}

#ifdef OBSOLETE
void adamNetwork::status(const FUJI_COMMAND_PACKET &packet)
{
    auto nstatus = NDevice::status(0);
    Debug_printf("adamNetwork::status avail=%d conn=%d err=%d\n",
                 nstatus.avail, nstatus.conn, nstatus.err);
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send(&nstatus, sizeof(nstatus), false);
}

void adamNetwork::set_query(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    NDevice::set_query(packet.dataAsString().value_or(""), 0);
    SYSTEM_BUS.transaction_success();
}
#endif /* OBSOLETE */

#endif /* BUILD_ADAM */
