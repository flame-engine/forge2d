import 'dart:typed_data';

import 'package:forge2d/src/backend/backend.dart';
import 'package:meta/meta.dart';

RawBox2D? _rawBox2D;

/// The active backend instance.
///
/// Internal to forge2d; not exported by the package.
RawBox2D get rawBox2D => _rawBox2D ??= createRawBox2D();

bool _worldHasBeenCreated = false;

/// Records that a world exists, which freezes the length unit.
///
/// Internal to forge2d; not exported by the package.
void markWorldCreated() => _worldHasBeenCreated = true;

/// Rounds [value] the way Box2D stores it, so that a length unit that was
/// handed to the native side compares equal when it is read back.
double _toFloat32(double value) => (Float32List(1)..[0] = value)[0];

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
/// [lengthUnitsPerMeter] tells Box2D how many of your length units make up
/// one meter, so that its internal tolerances line up with the scale your
/// world is laid out at. See `Tolerances` for what it scales. Prefer laying
/// the world out so that moving objects are roughly 0.1 to 10 meters over
/// reaching for this; it is the escape hatch for worlds that cannot be
/// scaled. Leaving it null keeps whatever value is in effect, which is 1
/// unless something else has set it.
///
/// The length unit is process-wide and cannot change once a `World` exists,
/// which is why it lives here rather than in a free-standing setter: this
/// call is the gate that already has to run before Box2D is touched. Passing
/// a value that conflicts with one that is already in effect throws a
/// [StateError] instead of silently corrupting the simulation. Passing the
/// value that is already in effect is always a no-op, so several games that
/// agree on a scale can each ask for it.
///
/// Cross-platform code should always call and await this first.
Future<void> initializeForge2D({
  Uri? wasmUri,
  double? lengthUnitsPerMeter,
}) async {
  await initializeBackend(wasmUri: wasmUri);
  final backend = _rawBox2D ??= createRawBox2D();
  if (lengthUnitsPerMeter == null) {
    return;
  }
  if (lengthUnitsPerMeter <= 0 || !lengthUnitsPerMeter.isFinite) {
    throw ArgumentError.value(
      lengthUnitsPerMeter,
      'lengthUnitsPerMeter',
      'must be positive and finite',
    );
  }
  final current = backend.getLengthUnitsPerMeter();
  if (current == _toFloat32(lengthUnitsPerMeter)) {
    return;
  }
  if (_worldHasBeenCreated) {
    throw StateError(
      'The length unit cannot be changed from $current to '
      '$lengthUnitsPerMeter because a World has already been created. Box2D '
      'bakes the length unit into the defaults of the definitions it hands '
      'out and into the tolerances of live simulations, so it has to be set '
      'before the first world exists. Await '
      'initializeForge2D(lengthUnitsPerMeter: ...) at startup, before any '
      'world is created.',
    );
  }
  backend.setLengthUnitsPerMeter(lengthUnitsPerMeter);
}

/// Forgets that a world has been created, so that a test can set a different
/// length unit than an earlier test did.
///
/// This only clears forge2d's bookkeeping. The value inside Box2D and any
/// world that is still alive are left alone, so destroy the worlds of the
/// previous test first.
@visibleForTesting
void debugResetLengthUnitLock() => _worldHasBeenCreated = false;
