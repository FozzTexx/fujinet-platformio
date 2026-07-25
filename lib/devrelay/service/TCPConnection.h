#pragma once

#if defined(DEV_RELAY_SLIP) && defined(SLIP_PROTOCOL_NET)

#include <memory>
#include <string>

#include "Connection.h"

class TCPConnection : public Connection, public std::enable_shared_from_this<TCPConnection>
{
public:
	TCPConnection(int socket) : socket_(socket) {}

	void send_data(const std::vector<uint8_t> &data) override;
	void create_read_channel() override;
	void close_connection() override;

	int get_socket() const { return socket_; }
	void set_socket(int socket) { this->socket_ = socket; }

private:
	int socket_;
};
#endif
