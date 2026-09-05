#include "Parser.h"

fujiError_t Parser::read(std::string &buffer, size_t length)
{
    fujiError_t err = _protocol->read(length);
    if (err != FUJI_ERROR::NONE)
        return err;
    buffer.resize(length);
    std::copy(_protocol->receiveBuffer->begin(),
              _protocol->receiveBuffer->begin() + buffer.size(), buffer.begin());
    _protocol->receiveBuffer->erase(0, buffer.size());
    _protocol->receiveBuffer->shrink_to_fit();
    return err;
}

fujiError_t Parser::write(std::string &buffer)
{
    fujiError_t err;

    std::string_view view(reinterpret_cast<const char*>(buffer.data()), buffer.size());
    *_protocol->transmitBuffer += view;
    return _protocol->write(view.size());
}

size_t Parser::available()
{
    return _protocol->available();
}

off_t Parser::seek(off_t offset, int whence)
{
    return _protocol->seek(offset, whence);
}

error_is_true Parser::setQuery(const std::string &query)
{
    // Nothing to parse, this is an error
    RETURN_ERROR_AS_TRUE();
}

error_is_true Parser::parse()
{
    // Nothing to parse, this is an error
    RETURN_ERROR_AS_TRUE();
}

NetworkStatus Parser::status()
{
    NetworkStatus ns;

    _protocol->status(&ns);
    return ns;
}

