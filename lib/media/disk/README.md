# lib/media/disk

Disk images in the `Media` hierarchy, read and written a block at a time while they're mounted.
RS232 mounts through these; the other buses still use their own `MediaType`.

## How it fits

- `DiskMedia` derives `Media` from [lib/media/media.h](../media.h). A drive names the formats it
  accepts in a `MediaFormats` list from [lib/media/](../) and opens them with
  `open<BlockMedia>()`, which tries each through `Media::open()` and its `attach()`. A drive
  that also takes cartridges tries `open<ROMMedia>()` first with the same handle, and closes the
  handle when nothing accepts it.
- A `BlockMedia` format with tracks describes them with `trackGeometry()` and places a sector with
  `offsetFromCHS()`; a flat image overrides `blockToOffset()` instead.

## Build

ESP: globbed by `src/CMakeLists.txt` into every target. PC: listed in `fujinet_pc.cmake` for every
target.
