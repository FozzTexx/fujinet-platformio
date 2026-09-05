#ifndef PARSER_H
#define PARSER_H

#include "Protocol.h"

class Parser
{
protected:
    NetworkProtocol *_protocol = nullptr;

public:
    Parser(NetworkProtocol *protocol) : _protocol(protocol) {}
    virtual ~Parser() = default;

    virtual fujiError_t read(std::string &buffer, size_t length);
    virtual fujiError_t write(std::string &buffer);
    virtual size_t available();
    virtual off_t seek(off_t offset, int whence);

    virtual error_is_true setQuery(const std::string &query);
    virtual error_is_true parse();

    virtual NetworkStatus status();
};

#endif /* PARSER_H */
