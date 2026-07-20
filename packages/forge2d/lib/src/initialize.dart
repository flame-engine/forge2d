import 'package:forge2d/src/backend/backend.dart';

RawBox2D? _rawBox2D;

/// The active backend instance.
///
/// Internal to forge2d; not exported by the package.
RawBox2D get rawBox2D => _rawBox2D ??= createRawBox2D();

/// Initializes forge2d.
///
/// On native platforms this completes immediately. On the web it fetches
/// and instantiates the Box2D WebAssembly module, and calling it before
/// creating any world is mandatory.
///
/// On the web the module is looked up at the package asset path served by
/// the Dart web tooling, at the package asset bundled into Flutter web
/// apps, and finally at `box2d.wasm` relative to the page. [wasmUri]
/// overrides the lookup for custom hosting setups.
///
/// Cross-platform code should always call and await this first.
Future<void> initializeForge2D({Uri? wasmUri}) async {
  await initializeBackend(wasmUri: wasmUri);
  _rawBox2D ??= createRawBox2D();
}
