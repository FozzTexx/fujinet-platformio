#ifndef SIO_AUDIO_H
#define SIO_AUDIO_H

#include <cassert>

#include "audioDevice.h"
#include "bus.h"

class sioAudio : public audioDevice
{
protected:
    void sio_status() override { audiocmd_status(); }
    void sio_process(uint32_t commanddata, uint8_t checksum) override;

public:
    void setup() override { audioDevice::setup(); }
};

extern sioAudio audioDev;

#endif // SIO_AUDIO_H
