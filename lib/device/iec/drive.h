#ifndef DRIVE_H
#define DRIVE_H

#include "../fuji/fujiHost.h"

#include <string>
#include <unordered_map>

#include "../../bus/bus.h"
#include "../../media/media.h"
#include "../meatloaf/meatloaf.h"
#include "../meatloaf/meat_buffer.h"
#include "../meatloaf/wrappers/iec_buffer.h"
#include "../meatloaf/wrappers/directory_stream.h"

#define PRODUCT_ID "MEATLOAF CBM"

class iecDrive : public virtualDevice
{
private:
    // /**
    //  * @brief the active bus protocol
    //  */
    // std::shared_ptr<DOS> _dos = nullptr;

    // /**
    //  * @brief Switch to detected bus protocol
    //  */
    // std::shared_ptr<DOS> selectDOS();

protected:
    //MediaType *_disk = nullptr;

    std::unique_ptr<MFile> _base;   // Always points to current directory/image
    std::string _last_file;         // Always points to last loaded file

    // RAM/ROM
//    std::streambuf ram;
    std::unique_ptr<MFile> rom;     // ROM File for current drive model if available

    // Directory
    uint16_t sendHeader(std::string header, std::string id);
    uint16_t sendLine(uint16_t blocks, char *text);
    uint16_t sendLine(uint16_t blocks, const char *format, ...);
    uint16_t sendFooter();
    void sendListing(int chan);

    // File
    bool sendFile(int chan);
    bool saveFile(int chan, IECPayload &payload);
    void sendFileNotFound();

    // Named Channel functions
    bool registerStream (uint8_t channel);
    std::shared_ptr<MStream> retrieveStream ( uint8_t channel );
    bool closeStream ( uint8_t channel, bool close_all = false );
    uint16_t retrieveLastByte ( uint8_t channel );
    void storeLastByte( uint8_t channel, char last);
    void flushLastByte( uint8_t channel );

    struct _error_response
    {
        unsigned char errnum = 73;
        std::string msg = "CBM DOS V2.6 1541";
        unsigned char track = 0;
        unsigned char sector = 0;
    } error_response;

    void read();
    void write(bool verify);
    void format();

protected:
    virtual bool openChannel(int chan, IECPayload &payload) override;
    virtual bool closeChannel(int chan) override;
    virtual bool readChannel(int chan) override;
    virtual bool writeChannel(int chan, IECPayload &payload) override;

    /**
     * @brief called to open a connection to a protocol
     */
    void iec_open(int chan, IECPayload &payload);

    /**
     * @brief called to process command either at open or listen
     */
    void iec_command(int chan, IECPayload &payload);

    /**
     * @brief If response queue is empty, Return 1 if ANY receive buffer has data in it, else 0
     */
    void iec_talk_command_buffer_status();

    /**
     * @brief Set device ID from dos command
     */
    void set_device_id();

    /**
     * @brief Set desired prefix for channel
     */
    void set_prefix(std::string payload);

    /**
     * @brief Get prefix for channel
     */
    void get_prefix();

public:
    iecDrive();
    fujiHost *host;
    mediatype_t mount(FILE *f, const char *filename, uint32_t disksize, mediatype_t disk_type = MEDIATYPE_UNKNOWN);
    void unmount();
    bool write_blank(FILE *f, uint16_t sectorSize, uint16_t numSectors);

    //mediatype_t disktype() { return _disk == nullptr ? MEDIATYPE_UNKNOWN : _disk->_mediatype; };

    std::unordered_map<uint16_t, std::shared_ptr<MStream>> streams;
    std::unordered_map<uint16_t, uint16_t> streamLastByte;

    ~iecDrive();
};

#endif // DRIVE_H
