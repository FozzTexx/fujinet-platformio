#include "ROMMedia.h"

class CoCoCartROM : public ROMMedia
{
protected:
    size_t _size;
    error_is_true attach() override;

public:
    CoCoCartROM(const std::string &path, fnFile *handle) : ROMMedia(path, handle) {}

    size_t size() const { return _size; }
    size_result_t read(ByteBuffer &buffer, size_t length) override;
    error_is_true close();
};
