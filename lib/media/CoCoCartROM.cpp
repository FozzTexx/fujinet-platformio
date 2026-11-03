#include "CoCoCartROM.h"

#include <strings.h>

#define COCO_CART_EXTENSION ".ccc"

error_is_true CoCoCartROM::attach()
{
    // FIXME - validate size

    size_t dot = _path.rfind('.');
    bool matches = dot != std::string::npos
        && strcasecmp(_path.c_str() + dot, COCO_CART_EXTENSION) == 0;
    if (!matches)
        RETURN_ERROR_AS_TRUE();

    _fileh->seek(0, SEEK_END);
    _size = _fileh->tell();
    _fileh->seek(0, SEEK_SET);
    RETURN_SUCCESS_AS_FALSE();
}

size_result_t CoCoCartROM::read(ByteBuffer &buffer, size_t length)
{
    if (_fileh == nullptr)
        RETURN_SIZE_ERROR();

    buffer.resize(length);
    size_t rlen = _fileh->read(buffer.data(), 1, buffer.size());
    buffer.resize(rlen);
    RETURN_SIZE(buffer.size());
}

error_is_true CoCoCartROM::close()
{
    if (!_fileh)
        RETURN_ERROR_AS_TRUE();
    _fileh->close();
    _fileh = nullptr;
    RETURN_SUCCESS_AS_FALSE();
}
