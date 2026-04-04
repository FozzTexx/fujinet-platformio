#ifndef _MEDIATYPE_DO_
#define _MEDIATYPE_DO_

#include <stdio.h>

#include "mediaType.h"

class MediaTypeDO : public MediaType
{
private:
    fujiError_t read_sector(int track, int sector, uint8_t* buffer);
    fujiError_t write_sector(int track, int sector, uint8_t* buffer) override;

public:
    fujiError_t read(uint32_t blockNum, uint16_t *count, uint8_t* buffer) override;
    fujiError_t write(uint32_t blockNum, uint16_t *count, uint8_t* buffer) override;

    fujiError_t format(uint16_t *responsesize) override;

    mediatype_t mount(fnFile *f, uint32_t disksize) override;

    fujiError_t status() override {return _media_fileh != nullptr
            ? FUJI_ERROR::NONE : FUJI_ERROR::UNSPECIFIED;}
};


#endif // _MEDIATYPE_DO_
