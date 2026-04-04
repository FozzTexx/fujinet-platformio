#ifndef _MEDIATYPE_DDP_
#define _MEDIATYPE_DDP_

#include <stdio.h>

#include "mediaType.h"

class MediaTypeDDP : public MediaType
{
private:
    uint32_t _block_to_offset(uint32_t blockNum);

public:
    virtual fujiError_t read(uint32_t blockNum, uint16_t *readcount) override;
    virtual fujiError_t write(uint32_t blockNum, bool verify) override;

    virtual fujiError_t format(uint16_t *responsesize) override;

    virtual mediatype_t mount(FILE *f, uint32_t disksize) override;

    virtual uint8_t status() override;

    static bool create(FILE *f, uint32_t numBlock);
};


#endif // _MEDIATYPE_DDP_
