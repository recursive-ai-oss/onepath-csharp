# glTF input

`GltfReader.Read(bytes)` turns a glTF 2.0 file into world-space triangles for the 3D builder (`NavMesh.BuildGltf`). The same file gives the
same triangles on every platform: node transforms are composed in double with `+ − ×` only (no trigonometry, no square roots).

## Files

- **`.glb`**: recognised by the magic `glTF` (little endian `0x46546C67`). Version must be 2; the header length must not exceed the file
  (a shorter file than its header says is an error; extra bytes after the stated length are ignored). Chunks run to the stated length,
  each padded to 4 bytes; the first JSON chunk and the first BIN chunk are used, other chunk types are ignored.
- **`.gltf`**: UTF-8 JSON, nesting at most 64 deep. Buffers are base64 data URIs, or any other URI answered by `GltfOptions.ResolveBuffer`
  (the reader never opens files). An unanswerable URI is an error naming it.
- `asset.version` must start with `2.`. `extensionsRequired` must be empty; `extensionsUsed` is ignored, extension data is not read.

## What is read

- **Scene:** `scene` if given, else scene 0, else (no `scenes`) every node that no node lists as a child. Nodes are walked depth first,
  children in listed order, with an explicit stack; depth above 256 (which includes cycles) is an error. A node is visited once per
  path to it (instancing), counted against `MaxNodes` (default 1 000 000).
- **Transform:** `matrix` (16 numbers, column-major) or any of `translation`, `rotation` (unit quaternion x, y, z, w; its squared
  length must be within 1e-3 of 1), `scale`, composed as T·R·S; both forms on one node is an error. World matrix = parent × local.
  Rotation matrix terms use `1 − 2(y²+z²)`, `2(xy ± zw)` and so on in this fixed operation order, then each column is scaled, then
  translation is added. A vertex is `M · (x, y, z, 1)` summed in the order x, y, z, 1.
- **Meshes:** each primitive's `POSITION` (float, VEC3, finite values) and optional `indices` (unsigned byte, short or int, scalar,
  each below the vertex count). Modes 4 (triangles), 5 (strip) and 6 (fan) are read; strips alternate winding (triangle *t* is
  *(t, t+1, t+2)* for even *t*, *(t+1, t, t+2)* for odd) and fans are *(0, t+1, t+2)*. Modes 0 to 3 are skipped; other modes are an error.
  Without `indices` the vertices are used in order.
- **Winding:** glTF's front face is counter-clockwise, which is the builder's convention (a floor seen from above). A node whose 3×3 has
  a negative determinant (a mirror) has the last two indices of every triangle swapped, as the specification requires.
- **Not read:** sparse accessors (error), normalized or integer positions (error), skins, animation, morph targets, materials, cameras.

## Accessor checks

Every accessor is resolved to a byte range before it is read: view offset + accessor offset + (count−1)·stride + element size must lie
inside the view, and the view inside the buffer, computed in 64 bits. Strides below the element size or above 252 are errors. A buffer
shorter than its `byteLength` is an error (a longer one, such as a padded GLB chunk, is fine).

## Limits and errors

`GltfOptions.MaxTriangles` (default 4 000 000, instances counted) and `MaxNodes` bound the work a small file can ask for. Every
failure is a `GltfException` whose message names the JSON path (`meshes[2].primitives[0]`, `accessors[5]`, …); a value of the wrong
JSON kind anywhere surfaces as a `GltfException` too. Coordinates beyond ±2^21 or non-finite after transformation are reported by the
builder, not the reader.
