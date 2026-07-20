#!/usr/bin/env bash
# Builds the Box2D WebAssembly module used by the web backend.
#
# Requires an activated emsdk (https://github.com/emscripten-core/emsdk);
# CI pins the version, see .github/workflows/build-wasm.yml.
#
# Usage: tool/build_wasm.sh
set -euo pipefail

PACKAGE_DIR="$(cd "$(dirname "$0")/.." && pwd)"
BOX2D="$PACKAGE_DIR/third_party/box2d"
OUT="$PACKAGE_DIR/lib/src/backend/wasm/box2d.wasm"

emcc \
  -O3 \
  -std=gnu17 \
  -DNDEBUG \
  -msimd128 \
  -msse2 \
  --no-entry \
  -sSTANDALONE_WASM=1 \
  -sALLOW_MEMORY_GROWTH=1 \
  -sINITIAL_MEMORY=16MB \
  -sFILESYSTEM=0 \
  -sEXPORTED_FUNCTIONS=_malloc,_free \
  -sERROR_ON_UNDEFINED_SYMBOLS=0 \
  -I"$BOX2D/include" \
  -I"$BOX2D/src" \
  "$BOX2D"/src/*.c \
  "$PACKAGE_DIR/native/wasm/f2d_shim.c" \
  -o "$OUT"

ls -la "$OUT"
