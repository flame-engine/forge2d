/// Compile-time selection of the platform backend.
///
/// Exports `createRawBox2D` and `initializeBackend`. The web backend will be
/// added here as a `dart.library.js_interop` branch.
library;

export 'package:forge2d/src/backend/raw_box2d.dart';
export 'package:forge2d/src/backend/raw_box2d_unsupported.dart'
    if (dart.library.ffi) 'package:forge2d/src/backend/raw_box2d_ffi.dart';
