#ifndef _MEDIATYPE_ATR_
#define _MEDIATYPE_ATR_

#include "diskType.h"

class MediaTypeATR : public MediaType
{
private:
    uint32_t _sector_to_offset(uint16_t sectorNum);

public:
    fujiError_t read(uint16_t sectornum, uint16_t *readcount) override;
    fujiError_t write(uint16_t sectornum, bool verify) override;

    fujiError_t format(uint16_t *responsesize) override;

    mediatype_t mount(fnFile *f, uint32_t disksize) override;

    void status(uint8_t statusbuff[4]) override;

    static fujiError_t create(fnFile *f, uint16_t sectorSize, uint16_t numSectors);
};


#endif // _MEDIATYPE_ATR_
