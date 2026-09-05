#ifndef SGMLPARSER_H
#define SGMLPARSER_H

#include "Parser.h"
#include "fnsgml.h" // FIXME - move all that in here

class SGMLParser : public Parser
{
protected:
    FNSGML _sgml;
#ifdef OBSOLETE
    unsigned short _remaining = 0;
#endif /* OBSOLETE */

public:
    using Parser::Parser;
    SGMLParser(NetworkProtocol *protocol) : Parser(protocol) {
        _sgml.setLineEnding("\x0a");
        _sgml.setProtocol(_protocol);
    }

#ifdef OBSOLETE
    fujiError_t read(std::string &buffer, size_t length) override;
#endif /* OBSOLETE */
    fujiError_t write(std::string &buffer) override;
#ifdef OBSOLETE
    size_t available() override;
#endif /* OBSOLETE */
    off_t seek(off_t offset, int whence) override;

    error_is_true setQuery(const std::string &query) override;
    error_is_true parse() override;

#ifdef OBSOLETE
    NetworkStatus status() override;
#endif /* OBSOLETE */
};

#endif /* SGMLPARSER_H */
