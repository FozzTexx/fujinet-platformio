#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include <doctest/doctest.h>

#include <algorithm>
#include <cstdio>
#include <vector>

#include "BlockMedia.h"

namespace
{

// Not tmpfile(): under MSYS2 that creates the file in the drive root.
#define BLOCK_MEDIA_TEST_FILE "blockmedia_test.tmp"

// Two cylinders, two sides. Track 0/0 has three 128-byte sectors, the rest
// two 256-byte sectors, stored track after track with side varying fastest.
class TrackedMedia : public BlockMedia
{
public:
    TrackedMedia(fnFile *handle, bool readOnly = false)
        : BlockMedia("tracked.img", handle), _readOnly(readOnly) {}

    using BlockMedia::blockToCHS;
    using BlockMedia::forgetPosition;

    uint8_t sideCount() const override { return 2; }
    uint16_t cylinderCount() const override { return 2; }

    TrackGeometry trackGeometry(uint16_t cylinder, uint8_t side) const override
    {
        if (cylinder == 0 && side == 0)
            return {3, 128};
        return {2, 256};
    }

    error_is_true close() override { RETURN_SUCCESS_AS_FALSE(); }
    bool isReadOnly() const override { return _readOnly; }

protected:
    error_is_true attach() override { RETURN_SUCCESS_AS_FALSE(); }

    error_is_true offsetFromCHS(const CHS &chs, uint32_t &offset) const override
    {
        offset = 0;
        for (uint16_t cylinder = 0; cylinder <= chs.cylinder; cylinder++)
            for (uint8_t side = 0; side < sideCount(); side++)
            {
                TrackGeometry geometry = trackGeometry(cylinder, side);
                if (cylinder == chs.cylinder && side == chs.side)
                {
                    offset += chs.sector * geometry.sectorSize;
                    RETURN_SUCCESS_AS_FALSE();
                }
                offset += geometry.sectorCount * geometry.sectorSize;
            }
        RETURN_ERROR_AS_TRUE();
    }

private:
    bool _readOnly;
};

// A 16-byte header and then four 64-byte blocks, with no track layout.
class FlatMedia : public BlockMedia
{
public:
    explicit FlatMedia(fnFile *handle) : BlockMedia("flat.img", handle) {}

    uint8_t sideCount() const override { return 1; }
    uint16_t cylinderCount() const override { return 1; }
    uint32_t blockCount() const override { return 4; }
    uint16_t blockSize(uint32_t block) const override { return block < 4 ? 64 : 0; }

    error_is_true close() override { RETURN_SUCCESS_AS_FALSE(); }
    bool isReadOnly() const override { return false; }

protected:
    error_is_true attach() override { RETURN_SUCCESS_AS_FALSE(); }

    error_is_true blockToOffset(uint32_t block, uint32_t &offset) const override
    {
        offset = 16 + block * 64;
        RETURN_SUCCESS_AS_FALSE();
    }
};

// Each byte holds its own offset, so a read shows where it came from.
std::vector<uint8_t> numbered(size_t size)
{
    std::vector<uint8_t> image(size);
    for (size_t idx = 0; idx < size; idx++)
        image[idx] = static_cast<uint8_t>(idx);
    return image;
}

FILE *image_file(const std::vector<uint8_t> &image)
{
    FILE *file = std::fopen(BLOCK_MEDIA_TEST_FILE, "w+b");
    REQUIRE(file != nullptr);
    REQUIRE(std::fwrite(image.data(), 1, image.size(), file) == image.size());
    return file;
}

std::vector<uint8_t> file_contents(FILE *file)
{
    std::fseek(file, 0, SEEK_END);
    std::vector<uint8_t> contents(std::ftell(file));
    std::fseek(file, 0, SEEK_SET);
    REQUIRE(std::fread(contents.data(), 1, contents.size(), file) == contents.size());
    return contents;
}

void remove_image(FILE *file)
{
    std::fclose(file);
    std::remove(BLOCK_MEDIA_TEST_FILE);
}

constexpr size_t TRACKED_SIZE = 3 * 128 + 3 * 2 * 256;

} // namespace

TEST_CASE("blockCount() adds up the sectors of every track")
{
    TrackedMedia media(nullptr);
    CHECK(media.blockCount() == 9);
}

TEST_CASE("blocks are numbered with side varying fastest")
{
    TrackedMedia media(nullptr);
    CHS chs;

    REQUIRE(media.blockToCHS(2, chs).is_success());
    CHECK(chs.cylinder == 0);
    CHECK(chs.side == 0);
    CHECK(chs.sector == 2);

    REQUIRE(media.blockToCHS(3, chs).is_success());
    CHECK(chs.cylinder == 0);
    CHECK(chs.side == 1);
    CHECK(chs.sector == 0);

    REQUIRE(media.blockToCHS(8, chs).is_success());
    CHECK(chs.cylinder == 1);
    CHECK(chs.side == 1);
    CHECK(chs.sector == 1);

    CHECK(media.blockToCHS(9, chs).is_error());
}

TEST_CASE("blockSize() is the size of the block's track, and 0 past the end")
{
    TrackedMedia media(nullptr);
    CHECK(media.blockSize(0) == 128);
    CHECK(media.blockSize(3) == 256);
    CHECK(media.blockSize(9) == 0);
}

TEST_CASE("a block reads from where its CHS puts it")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);
    ByteBuffer buffer;

    REQUIRE(media.readBlock(1, buffer).is_success());
    CHECK(buffer == ByteBuffer(image.begin() + 128, image.begin() + 256));

    REQUIRE(media.readBlock(5, buffer).is_success());
    CHECK(buffer.size() == 256);
    CHECK(buffer == ByteBuffer(image.begin() + 384 + 512, image.begin() + 384 + 768));

    CHECK(media.readBlock(9, buffer).is_error());
    remove_image(file);
}

TEST_CASE("a written block lands at its offset and nowhere else")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);

    ByteBuffer data(256, 0xA5);
    REQUIRE(media.writeBlock(4, data).is_success());

    std::copy(data.begin(), data.end(), image.begin() + 384 + 256);
    CHECK(file_contents(file) == image);
    remove_image(file);
}

TEST_CASE("a write of the wrong size, past the end, or to read-only media is refused")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);

    TrackedMedia media(file);
    CHECK(media.writeBlock(0, ByteBuffer(256, 0)).is_error());
    CHECK(media.writeBlock(9, ByteBuffer(256, 0)).is_error());

    TrackedMedia readOnly(file, true);
    CHECK(readOnly.writeBlock(0, ByteBuffer(128, 0)).is_error());

    CHECK(file_contents(file) == image);
    remove_image(file);
}

TEST_CASE("media with no handle refuses reads and writes")
{
    TrackedMedia media(nullptr);
    ByteBuffer buffer;
    CHECK(media.readBlock(0, buffer).is_error());
    CHECK(media.writeBlock(0, ByteBuffer(128, 0)).is_error());
}

TEST_CASE("a flat image reads and writes through its own blockToOffset()")
{
    auto image = numbered(16 + 4 * 64);
    FILE *file = image_file(image);
    FlatMedia media(file);
    ByteBuffer buffer;

    REQUIRE(media.readBlock(2, buffer).is_success());
    CHECK(buffer == ByteBuffer(image.begin() + 16 + 128, image.begin() + 16 + 192));

    ByteBuffer data(64, 0x5A);
    REQUIRE(media.writeBlock(3, data).is_success());
    std::copy(data.begin(), data.end(), image.begin() + 16 + 192);
    CHECK(file_contents(file) == image);

    CHECK(media.readBlock(4, buffer).is_error());
    remove_image(file);
}

// The handle is moved behind the media's back, so a read that skips its seek
// shows it by returning the bytes at the moved position.
TEST_CASE("the block after the last one read is read without a seek")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);
    ByteBuffer buffer;

    REQUIRE(media.readBlock(0, buffer).is_success());
    std::fseek(file, 0, SEEK_SET);
    REQUIRE(media.readBlock(1, buffer).is_success());
    CHECK(buffer == ByteBuffer(image.begin(), image.begin() + 128));
    remove_image(file);
}

TEST_CASE("a block out of order, or after forgetPosition(), is sought")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);
    ByteBuffer buffer;

    REQUIRE(media.readBlock(0, buffer).is_success());
    std::fseek(file, 0, SEEK_SET);
    REQUIRE(media.readBlock(2, buffer).is_success());
    CHECK(buffer == ByteBuffer(image.begin() + 256, image.begin() + 384));

    std::fseek(file, 0, SEEK_SET);
    media.forgetPosition();
    REQUIRE(media.readBlock(3, buffer).is_success());
    CHECK(buffer == ByteBuffer(image.begin() + 384, image.begin() + 640));
    remove_image(file);
}

TEST_CASE("a write right after a read of the block before it still seeks")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);
    ByteBuffer buffer;

    REQUIRE(media.readBlock(0, buffer).is_success());
    std::fseek(file, 0, SEEK_SET);
    ByteBuffer data(128, 0xC3);
    REQUIRE(media.writeBlock(1, data).is_success());

    std::copy(data.begin(), data.end(), image.begin() + 128);
    CHECK(file_contents(file) == image);
    remove_image(file);
}

TEST_CASE("consecutive writes land in order")
{
    auto image = numbered(TRACKED_SIZE);
    FILE *file = image_file(image);
    TrackedMedia media(file);

    ByteBuffer first(256, 0x11);
    ByteBuffer second(256, 0x22);
    REQUIRE(media.writeBlock(3, first).is_success());
    REQUIRE(media.writeBlock(4, second).is_success());

    std::copy(first.begin(), first.end(), image.begin() + 384);
    std::copy(second.begin(), second.end(), image.begin() + 640);
    CHECK(file_contents(file) == image);
    remove_image(file);
}
