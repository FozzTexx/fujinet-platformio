#ifndef FN_FILEMEM_H
#define FN_FILEMEM_H

#include <stdint.h>
#include <cstddef>

#include "fnFile.h"

#define FILEMEM_MAXSIZE   1048576

class FileHandlerMem : public FileHandler
{
protected:
    uint8_t *_buffer;
    long int _size;
    long int _filesize;
    long int _position;
public:
    FileHandlerMem();
    ~FileHandlerMem() override;

    int close(bool destroy=true) override;
    int seek(long int off, int whence) override;
    long int tell() override;
    size_t read(void *ptr, size_t size, size_t count) override;
    size_t write(const void *ptr, size_t size, size_t count) override;
    int flush() override;

    int grow(long filesize);
};

#endif // FN_FILEMEM_H
