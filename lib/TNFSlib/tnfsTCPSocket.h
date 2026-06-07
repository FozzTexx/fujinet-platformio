#ifndef _TNFS_TCP_SOCKET_H_
#define _TNFS_TCP_SOCKET_H_

#include <cstring>
#include <errno.h>
#include "compat_inet.h"
#include "fnDNS.h"
#include "../../include/debug.h"

#ifdef ESP_PLATFORM
#  include <lwip/sockets.h>
#else
#  include <sys/socket.h>
#  include <netinet/in.h>
#  include <unistd.h>
#  define closesocket close
#endif

class tnfsTCPSocket
{
private:
    int _fd = -1;

    bool _setRecvTimeout(int32_t timeout_ms)
    {
        struct timeval tv;
        tv.tv_sec  = timeout_ms / 1000;
        tv.tv_usec = (timeout_ms % 1000) * 1000;
        if (setsockopt(_fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0)
        {
            Debug_printf("tnfsTCPSocket: SO_RCVTIMEO failed: %d\r\n", errno);
            return false;
        }
        return true;
    }

    bool _connectFd(in_addr_t addr, uint16_t port, int32_t timeout_ms)
    {
        stop();

        _fd = socket(AF_INET, SOCK_STREAM, 0);
        if (_fd < 0)
        {
            Debug_printf("tnfsTCPSocket: socket() failed: %d\r\n", errno);
            return false;
        }

        struct sockaddr_in serveraddr;
        memset(&serveraddr, 0, sizeof(serveraddr));
        serveraddr.sin_family      = AF_INET;
        serveraddr.sin_addr.s_addr = addr;
        serveraddr.sin_port        = htons(port);

        // Set connect timeout via SO_RCVTIMEO before connect
        if (timeout_ms > 0)
            _setRecvTimeout(timeout_ms);

        if (::connect(_fd, (struct sockaddr *)&serveraddr, sizeof(serveraddr)) < 0)
        {
            Debug_printf("tnfsTCPSocket: connect() failed: %d\r\n", errno);
            stop();
            return false;
        }

        Debug_printf("tnfsTCPSocket: connected to %s:%u\r\n", inet_ntoa(serveraddr.sin_addr), port);
        return true;
    }

public:
    tnfsTCPSocket() {}
    ~tnfsTCPSocket() { stop(); }

    bool connect(in_addr_t addr, uint16_t port, int32_t timeout_ms = 5000)
    {
        return _connectFd(addr, port, timeout_ms);
    }

    bool connect(const char *host, uint16_t port, int32_t timeout_ms = 5000)
    {
        in_addr_t addr = get_ip4_addr_by_name(host);
        if (addr == IPADDR_NONE)
        {
            Debug_printf("tnfsTCPSocket: could not resolve host \"%s\"\r\n", host);
            return false;
        }
        return _connectFd(addr, port, timeout_ms);
    }

    void stop()
    {
        if (_fd >= 0)
        {
            closesocket(_fd);
            _fd = -1;
        }
    }

    bool connected() const
    {
        return _fd >= 0;
    }

    // Set receive timeout in milliseconds. Call after connect.
    bool setTimeout(int32_t timeout_ms)
    {
        if (_fd < 0) return false;
        return _setRecvTimeout(timeout_ms);
    }

    // Write exactly size bytes. Returns size on success, -1 on error.
    int write(const uint8_t *buf, size_t size)
    {
        if (_fd < 0) return -1;
        int sent = 0;
        while (sent < (int)size)
        {
            int r = ::send(_fd, buf + sent, size - sent, 0);
            if (r < 0)
            {
                Debug_printf("tnfsTCPSocket: send() failed: %d\r\n", errno);
                return -1;
            }
            sent += r;
        }
        return sent;
    }

    // Read up to size bytes. Blocks until data arrives or SO_RCVTIMEO fires.
    // Returns bytes read, 0 on timeout, -1 on error/disconnect.
    int read(uint8_t *buf, size_t size)
    {
        if (_fd < 0) return -1;
        int r = ::recv(_fd, buf, size, 0);
        if (r < 0)
        {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return 0; // timeout
            Debug_printf("tnfsTCPSocket: recv() failed: %d\r\n", errno);
            return -1;
        }
        if (r == 0)
        {
            Debug_printf("tnfsTCPSocket: connection closed by server\r\n");
            stop();
            return -1;
        }
        return r;
    }

    int fd() const { return _fd; }

    in_addr_t remoteIP() const
    {
        if (_fd < 0) return IPADDR_NONE;
        struct sockaddr_in addr;
        socklen_t len = sizeof(addr);
        if (getpeername(_fd, (struct sockaddr *)&addr, &len) < 0)
            return IPADDR_NONE;
        return addr.sin_addr.s_addr;
    }
};

#endif // _TNFS_TCP_SOCKET_H_
