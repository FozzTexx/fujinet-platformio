#ifndef ROMMEDIA_H
#define ROMMEDIA_H

#include "Media.h"

#include <cstdint>
#include <memory>
#include <vector>

class ROMMedia : public Media
{
public:
    explicit ROMMedia(const std::string &path, fnFile *handle) : Media(path, handle) {}

    bool isReadOnly() const override { return true; }

    // Image length in bytes, excluding any format header.
    virtual size_t size() const = 0;

    // Returns the next chunk of the image; a short or empty buffer means
    // the end. An error is reserved for I/O failure.
    virtual size_result_t read(ByteBuffer &buffer, size_t length) = 0;

    // Opaque mapper description passed through to the device verbatim;
    // nullptr if the format has none.
    virtual std::unique_ptr<std::vector<uint8_t>> mapperInfo() { return nullptr; }

    // Save data is determined by the media, but is deferred for now.
};

#endif /* ROMMEDIA_H */
