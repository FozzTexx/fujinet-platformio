#include "Protocol.h"

class Parser
{
protected:
    NetworkProtocol *_protocol = nullptr;

public:
    Parser(NetworkProtocol *protocol) : _protocol(protocol) {}
    virtual ~Parser() = default;

    fujiError_t read(std::string &buffer, size_t length);
    fujiError_t write(std::string &buffer);
    size_t available();
    off_t seek(off_t offset, int whence);

    error_is_true setQuery(const std::string &query);
    error_is_true parse();

    NetworkStatus status();
};
