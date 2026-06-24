#ifdef BUILD_ATARI

#include "audio.h"

#include "fujiCommandID.h"

sioAudio audioDev;

void sioAudio::sio_process(uint32_t commanddata, uint8_t checksum)
{
    cmdFrame.commanddata = commanddata;
    cmdFrame.checksum = checksum;

    switch (cmdFrame.comnd)
    {
    case AUDIOCMD_STATUS:
        audiocmd_status();
        break;
    case AUDIOCMD_CAPABILITIES:
        audiocmd_capabilities(cmdFrame.aux1);
        break;
    case AUDIOCMD_SET_SOURCE:
        audiocmd_set_source(cmdFrame.aux1 | (cmdFrame.aux2 << 8));
        break;
    case AUDIOCMD_PLAY:
        audiocmd_play(cmdFrame.aux1);
        break;
    case AUDIOCMD_PAUSE:
        audiocmd_pause();
        break;
    case AUDIOCMD_RESUME:
        audiocmd_resume();
        break;
    case AUDIOCMD_STOP:
        audiocmd_stop();
        break;
    case AUDIOCMD_SET_VOLUME:
        audiocmd_set_volume(cmdFrame.aux1, true);
        break;
    case AUDIOCMD_GET_INFO:
        audiocmd_get_info();
        break;
    case AUDIOCMD_GET_METADATA:
        audiocmd_get_metadata(cmdFrame.aux1, cmdFrame.aux2);
        break;
    case AUDIOCMD_SEEK:
        audiocmd_seek();
        break;
    default:
        transaction_error();
        break;
    }
}

#endif // BUILD_ATARI
