#ifndef JSONPARSER_H
#define JSONPARSER_H

#include "Parser.h"
#include "fnjson.h" // FIXME - move all that in here

class JSONParser : public Parser
{
protected:
    FNJSON _json;

public:
    using Parser::Parser;
    JSONParser(NetworkProtocol *protocol) : Parser(protocol) {
        _json.setLineEnding("\x0a");
        _json.setProtocol(_protocol);
    }

    fujiError_t write(std::string &buffer) override;
    off_t seek(off_t offset, int whence) override;

    error_is_true setQuery(const std::string &query) override;
    error_is_true parse() override;
};

#endif /* JSONPARSER_H */
