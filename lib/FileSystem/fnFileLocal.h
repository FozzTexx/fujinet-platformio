#ifndef FN_FILELOCAL_H
#define FN_FILELOCAL_H

#include <cstdio>
#include "fnFile.h"


class FileHandlerLocal : public FileHandler
{
protected:
    FILE *_fh = nullptr;

public:
    FileHandlerLocal(FILE *fh);
    ~FileHandlerLocal() override;

    int close(bool destroy=true) override;
    int seek(long int off, int whence) override;
    long int tell() override;
    size_t read(void *ptr, size_t size, size_t n) override;
    size_t write(const void *ptr, size_t size, size_t n) override;
    int flush() override;
};


#endif // FN_FILELOCAL_H
