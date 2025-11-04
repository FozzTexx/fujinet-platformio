#ifndef NETSIO_H
#define NETSIO_H

//#define NETSIO_OBSOLETE 1

#include "fnDNS.h"
#include <string.h>
#include <string>
#include <sys/time.h>

#define NETSIO_PORT             9997
#define SIOPORT_DEFAULT_BAUD   19200

namespace SioCom {
    enum sio_mode
    {
        SERIAL = 0,
        NETSIO
    };
}

class NetSIO // : public SioPort
{
private:
    char _host[64] = {0};
    in_addr_t _ip = IPADDR_NONE;
    uint16_t _port = NETSIO_PORT;

    uint32_t _baud = SIOPORT_DEFAULT_BAUD;
    uint32_t _baud_peer = SIOPORT_DEFAULT_BAUD;
    int _fd = -1;
    bool _initialized = false;
    bool _command_asserted = false;
    bool _motor_asserted = false;

#ifdef NETSIO_OBSOLETE
    uint8_t _rxbuf[1024];
    int _rxhead = 0;
    int _rxtail = 0;
    bool _rxfull = false;
#else
    std::string _fifo;
#endif /* NETSIO_OBSOLETE */

    int _sync_request_num = -1;  // 0..255 sync request sequence number, -1 if sync is not requested
    uint8_t _sync_ack_byte = -1; // ACK byte to send with sync response
    int _sync_write_size;   // 0 .. no SIO write (from computer), > 0 .. expected bytes written

    // serial port error counter
    int _errcount = 0;
    uint64_t _resume_time;
    uint64_t _alive_time;    // when last message was received
    uint64_t _alive_request; // when last ALIVE request was sent
    // flow control
    int _credit = 3;

    size_t _print_number(unsigned long n, uint8_t base);

protected:
    void suspend(int ms=5000);
    bool resume_test();
    bool keep_alive();

    int handle_netsio();
    static timeval timeval_from_ms(const uint32_t millis);

    bool wait_sock_readable(uint32_t timeout_ms);
    bool wait_for_data(uint32_t timeout_ms);
    bool wait_for_credit(int needed);

    bool wait_sock_writable(uint32_t timeout_ms);
    ssize_t write_sock(const uint8_t *buffer, size_t size, uint32_t timeout_ms=500);

    bool rxbuffer_empty();
    bool rxbuffer_put(uint8_t b);
    int rxbuffer_get();
    int rxbuffer_available();
    void rxbuffer_flush();

public:
    virtual ~NetSIO();
    void begin(int baud);
    void end();
    bool poll(int ms);

    void set_baudrate(uint32_t baud);
    uint32_t get_baudrate();

    bool command_asserted();
    bool motor_asserted();
    void set_proceed(bool level);
    void set_interrupt(bool level);

    void bus_idle(uint16_t ms);

    int available();
    void flush();
    void flush_input();

    // read single byte
    int read();
    // read bytes into buffer
    size_t read(uint8_t *buffer, size_t length);

    // write single byte
    ssize_t write(uint8_t b);
    // write buffer
    ssize_t write(const uint8_t *buffer, size_t size);

    size_t print(long n, int base = 10);
    size_t print(int n, int base = 10) { return print((long) n, base); }
    size_t print(const char *str) { return write((const uint8_t *) str, strlen(str)); }
    size_t print(const std::string &str) { return print(str.c_str()); }

    // specific to NetSioPort
    void set_host(const char *host, int port);
    const char* get_host(int &port);
    int ping(int count=4, int interval_ms=1000, int timeout_ms=500, bool fast=true);

    void set_sync_ack_byte(int ack_byte);
    void set_sync_write_size(int write_size);
    ssize_t send_sync_response(uint8_t response_type, uint8_t ack_byte=0, uint16_t sync_write_size=0);
    void send_empty_sync();

    SioCom::sio_mode get_sio_mode() { return SioCom::NETSIO; }
};

#endif // NETSIO_H
