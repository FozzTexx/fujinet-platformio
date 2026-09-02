#ifdef BUILD_RS232

#include "rs232Network.h"

#ifdef OBSOLETE
void rs232Network::fujidev_status(const FUJI_COMMAND_PACKET &packet)
{
    FujiStatusReq reqType = STATREQ::CONNERR;
    if (packet.paramCount() >= 2)
        reqType = (FujiStatusReq) packet.param(1);

    auto nstatus = NDevice::fujicore_status(static_cast<uint8_t>(reqType));
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    SYSTEM_BUS.transaction_send(&nstatus, sizeof(nstatus));
}
#endif /* OBSOLETE */

// RS232 uses variable length queries
void rs232Network::fujidev_set_query(const FUJI_COMMAND_PACKET &packet)
{
    SYSTEM_BUS.transaction_accept(TRANS_STATE::NO_GET);
    NDevice::fujicore_set_query(packet.dataAsString().value_or(""), 0);
    SYSTEM_BUS.transaction_success();
}

#endif /* BUILD_RS232 */
