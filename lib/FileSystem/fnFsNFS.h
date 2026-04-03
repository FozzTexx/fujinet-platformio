#ifndef FN_FSNFS_H
#define FN_FSNFS_H

#include <stdint.h>
#include <cstddef>
#include <nfsc/libnfs.h>

#include "fnFS.h"
#include "fnDirCache.h"


class FileSystemNFS : public FileSystem
{
private:
    struct nfs_context *_nfs;
    struct nfs_url *_url;

    // directory cache
    char _last_dir[MAX_PATHLEN];
    DirCache _dircache;

public:
    FileSystemNFS();
    ~FileSystemNFS();

    fujiError_t start(const char *url, const char *user=nullptr, const char *password=nullptr);

    fsType type() override { return FSTYPE_NFS; };
    const char *typestring() override { return type_to_string(FSTYPE_NFS); };

    FILE *file_open(const char *path, const char *mode = FILE_READ) override;
#ifndef FNIO_IS_STDIO
    FileHandler *filehandler_open(const char *path, const char *mode = FILE_READ) override;
#endif

    bool exists(const char *path) override;

    fujiError_t remove(const char *path) override;

    fujiError_t rename(const char *pathFrom, const char *pathTo) override;

    bool is_dir(const char *path) override;
    fujiError_t mkdir(const char* path) override { return FUJI_ERROR::UNSPECIFIED; };
    fujiError_t rmdir(const char* path) override { return FUJI_ERROR::UNSPECIFIED; };
    bool dir_exists(const char* path) override { return true; };

    fujiError_t dir_open(const char *path, const char *pattern, uint16_t diropts) override;
    fsdir_entry *dir_read() override;
    void dir_close() override;
    uint16_t dir_tell() override;
    fujiError_t dir_seek(uint16_t pos) override;
};

#endif // FN_FSNFS_H
