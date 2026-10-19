// Bindings to native/ffi/f2d_world_lock.c, which the build hook compiles
// into the same library as Box2D.
@ffi.DefaultAsset('package:forge2d/src/ffi/box2d.g.dart')
library;

import 'dart:ffi' as ffi;

import 'package:forge2d/src/ffi/box2d.g.dart';

/// `b2CreateWorld` under a process-wide lock.
@ffi.Native<b2WorldId Function(ffi.Pointer<b2WorldDef>)>(
  symbol: 'f2d_create_world_locked',
)
external b2WorldId f2dCreateWorldLocked(
  ffi.Pointer<b2WorldDef> definition,
);

/// `b2DestroyWorld` under the same lock as [f2dCreateWorldLocked].
@ffi.Native<ffi.Void Function(b2WorldId)>(symbol: 'f2d_destroy_world_locked')
external void f2dDestroyWorldLocked(b2WorldId id);

/// `b2SetLengthUnitsPerMeter` under the same lock as [f2dCreateWorldLocked],
/// refused once any isolate has created a world.
///
/// Returns whether the length unit was set.
@ffi.Native<ffi.Bool Function(ffi.Float)>(
  symbol: 'f2d_set_length_units_per_meter_locked',
)
external bool f2dSetLengthUnitsPerMeterLocked(double lengthUnits);

/// Forgets that a world has been created, so that the length unit can be set
/// again.
@ffi.Native<ffi.Void Function()>(symbol: 'f2d_reset_length_unit_lock')
external void f2dResetLengthUnitLock();
