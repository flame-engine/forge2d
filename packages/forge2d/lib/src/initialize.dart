import 'package:forge2d/src/backend/backend.dart';

RawBox2D? _rawBox2D;

/// The active backend instance.
///
/// Internal to forge2d; not exported by the package.
RawBox2D get rawBox2D => _rawBox2D ??= createRawBox2D();

/// Initializes forge2d.
///
/// On native platforms this completes immediately. On the web (in a future
/// release) it loads and instantiates the Box2D WebAssembly module, and
/// calling it before creating any world is mandatory.
///
/// Cross-platform code should always call and await this first.
Future<void> initializeForge2D() async {
  await initializeBackend();
  _rawBox2D ??= createRawBox2D();
}
