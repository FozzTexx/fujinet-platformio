#ifndef _MEDIATYPE_IMG
#define _MEDIATYPE_IMG

#include "diskType.h"

class MediaTypeImg : public MediaType
{
private:
    uint32_t _sector_to_offset(uint32_t sectorNum);

public:
    fujiError_t read(uint32_t sectornum, uint32_t *readcount) override;
    fujiError_t write(uint32_t sectornum, bool verify) override;

    fujiError_t format(uint32_t *responsesize) override;

    mediatype_t mount(fnFile *f, uint32_t disksize) override;

    void status(uint8_t statusbuff[4]) override;

    static fujiError_t create(fnFile *f, uint16_t sectorSize, uint32_t numSectors);
};


#endif // _MEDIATYPE_IMG
