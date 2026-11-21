# lib/media

Disk, tape and ROM image formats, one directory per platform. A media class knows how to read and
write sectors or blocks of one file format; it knows nothing about the bus (a ctest fails any
`SYSTEM_BUS` reference under this directory) and reaches files only through `fnio` or `FILE`.

## Platforms

There is no shared `MediaType` base class. Each directory declares its own `MediaType` and
`mediatype_t` enum, with a `mount()`, `read()`, `write()`, `status()` family shaped for that
bus (sector or block, `fnFile` or `FILE`) and a static `discover_mediatype(filename)` that maps a
file extension to the enum. RS232 (`BUILD_RS232`) has no directory here: its disk formats are in
[disk/](disk/) and its cartridges in [rom/](rom/).

| Directory | Platform | Base header | Formats | ESP | PC |
|---|---|---|---|---|---|
| [atari/](atari/) | `BUILD_ATARI` | `atari/diskType.h` | ATR, ATX, XEX (COM, BIN); CAS and WAV tape | yes | yes |
| [apple/](apple/) | `BUILD_APPLE` | `apple/mediaType.h` | PO (HDV, 2MG), DO, WOZ, DSK | yes | yes |
| [adam/](adam/) | `BUILD_ADAM`, also `BUILD_S100` | `adam/mediaType.h` | DDP, DSK | yes | yes |
| [drivewire/](drivewire/) | `BUILD_COCO` | `drivewire/mediaType.h` | DSK, MRM, VDK, CAS presented as a disk; cartridges are `CoCoCartROM` in [rom/](rom/) | yes | yes |
| [lynx/](lynx/) | `BUILD_LYNX` | `lynx/mediaType.h` | ROM | yes | yes |
| [mac/](mac/) | `BUILD_MAC` | `mac/mediaType.h` | MOOF, DCD (HD20), DiskCopy and raw floppy images, StuffIt archives | yes | no |
| [cbm/](cbm/) | `BUILD_IEC` | `cbm/mediaType.h` | type detection only; images are handled by `lib/meatloaf` | yes | no |
| [cx16/](cx16/) | `BUILD_CX16` | `cx16/mediaType.h` | none | legacy | no |
| [rc2014/](rc2014/) | `BUILD_RC2014` | `rc2014/mediaType.h` | IMG (RomWBW geometries) | legacy | no |
| [s100spi/](s100spi/) | none | `s100spi/mediaType.h` | DSK | unreachable: `media.h` gives `BUILD_S100` the `adam/` headers | no |

## How it fits

- `DISK_DEVICE::mount()` in [lib/device/](../device/) calls the platform's `discover_mediatype()`
  and instantiates the matching subclass; `fujiDevice::fujicore_mount_disk_image_success()` in
  [lib/device/fujiDevice/](../device/fujiDevice/) drives that from a `fujiDisk` slot.
- A format in [disk/](disk/) or [rom/](rom/) declares the `extensions` it accepts. Each class
  that mounts media pairs the formats it takes with the platform's `mediatype_t` IDs in a
  `MediaFormats` list (`MediaFormats.h`). The platform's Fuji device joins those lists with
  `JoinedMediaFormats` to type a directory entry, so each format is named once.
- The file handle comes from `fujiHost::fnfile_open()` in [lib/fuji/](../fuji/), so a media
  class works the same over SD, TNFS, SMB, NFS, FTP or HTTP.
- Big buffers (ATX tracks, directory caches, IMD images) use `PSRAMAllocator` from
  [include/](../../include/) on the ESP32.

## Build

ESP: `src/CMakeLists.txt` globs `lib/media/*.cpp` and `lib/media/**/*.cpp` into every target;
each platform file guards itself with `#ifdef BUILD_*`. `IMDImage.cpp` and `mac/macGCR.cpp` are
unguarded. PC: `fujinet_pc.cmake` compiles `IMDImage` for every target and one platform
directory per `FUJINET_TARGET`. `tests/` covers `IMDImage`, the [disk/](disk/) formats, `macGCR`,
and the Atari cassette helpers.

## Notes

- `IMDImage` uses only the standard library so a host tool can copy it, and is deliberately not a
  `Media`; `IMDDisk` in [disk/](disk/) wraps it as one.
