/// Compile-time selection of the platform backend.
///
/// Exports `createRawBox2D` and `initializeBackend`: dart:ffi against the
/// native library on native platforms, dart:js_interop against the bundled
/// box2d.wasm on the web.
library;

export 'package:forge2d/src/backend/raw_box2d.dart';
export 'package:forge2d/src/backend/raw_box2d_unsupported.dart'
    if (dart.library.ffi) 'package:forge2d/src/backend/raw_box2d_ffi.dart'
    if (dart.library.js_interop) 'package:forge2d/src/backend/raw_box2d_wasm.dart';
