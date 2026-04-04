#ifndef _MEDIATYPE_ROM_
#define _MEDIATYPE_ROM_

#include <stdio.h>

#include "mediaType.h"


class MediaTypeROM : public MediaType
{
    uint32_t _block_to_offset(uint32_t blockNum);

public:
    fujiError_t read(uint32_t blockNum, uint16_t *readcount) override;
    fujiError_t write(uint32_t blockNum, bool verify) override;

    fujiError_t format(uint16_t *responsesize) override;

    mediatype_t mount(FILE *f, uint32_t disksize) override;

    uint8_t status() override;

    static fujiError_t create(FILE *f, uint32_t numBlock);
};


#endif // _MEDIATYPE_ROM_
