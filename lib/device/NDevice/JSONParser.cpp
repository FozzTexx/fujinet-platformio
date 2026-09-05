#include "JSONParser.h"

fujiError_t JSONParser::read(std::string &buffer, size_t length)
{
    fujiError_t err = Parser::read(buffer, std::min<size_t>(_remaining, length));
    if (err == FUJI_ERROR::NONE)
        _remaining -= length;
    return err;
}

fujiError_t JSONParser::write(std::string &buffer)
{
    return FUJI_ERROR::UNSPECIFIED;
}

size_t JSONParser::available()
{
    return _remaining;
}

off_t JSONParser::seek(off_t offset, int whence)
{
    return -1;
}

error_is_true JSONParser::setQuery(const std::string &query)
{
    std::string buffer;

    _json.setReadQuery(query, 0);
    _remaining = _json.available();
    buffer.resize(_remaining);
    _json.readValue(reinterpret_cast<uint8_t *>(buffer.data()), _remaining);
    buffer.resize(strlen(buffer.c_str()));
    buffer = SYSTEM_BUS.unicodeTextToNative(buffer);
    _remaining = buffer.size();
    *_protocol->receiveBuffer += buffer;
    RETURN_SUCCESS_AS_FALSE();
}

error_is_true JSONParser::parse()
{
    _json.parse();
    RETURN_SUCCESS_AS_FALSE();
}

NetworkStatus JSONParser::status()
{
    NetworkStatus ns;

    ns.connected = _remaining > 0;
    ns.error = _remaining > 0 ? NDEV_STATUS::SUCCESS : NDEV_STATUS::END_OF_FILE;
    return ns;
}
