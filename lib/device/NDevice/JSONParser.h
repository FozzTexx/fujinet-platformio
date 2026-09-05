#ifndef JSONPARSER_H
#define JSONPARSER_H

#include "Parser.h"
#include "fnjson.h" // FIXME - move all that in here

class JSONParser : public Parser
{
protected:
    FNJSON _json;
    unsigned short _remaining = 0;

public:
    using Parser::Parser;
    JSONParser(NetworkProtocol *protocol) : Parser(protocol) {
        _json.setLineEnding("\x0a");
        _json.setProtocol(_protocol);
    }

    fujiError_t read(std::string &buffer, size_t length) override;
    fujiError_t write(std::string &buffer) override;
    size_t available() override;
    off_t seek(off_t offset, int whence) override;

    error_is_true setQuery(const std::string &query) override;
    error_is_true parse() override;

    NetworkStatus status() override;
};

#endif /* JSONPARSER_H */
