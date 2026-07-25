#ifndef FN_FILETNFS_H
#define FN_FILETNFS_H

#include <stdlib.h>

#include "tnfslib.h"
#include "fnFile.h"


class FileHandlerTNFS : public FileHandler
{
protected:
    tnfsMountInfo *_mountinfo = nullptr;
    int _handle = -1;

private:
    uint8_t _bad_fd_recovery();

public:
    FileHandlerTNFS(tnfsMountInfo *mountinfo, int handle);
    ~FileHandlerTNFS() override;

    int close(bool destroy=true) override;
    int seek(long int off, int whence) override;
    long int tell() override;
    size_t read(void *ptr, size_t size, size_t count) override;
    size_t write(const void *ptr, size_t size, size_t count) override;
    int flush() override;
};


#endif // FN_FILETNFS_H
