/// The low-level backend contract of forge2d.
///
/// The public API layer talks to the native library exclusively through this
/// interface. There is one implementation per platform mechanism: dart:ffi
/// against the native library, and (in a future release) JS interop against a
/// WebAssembly build.
///
/// The contract is deliberately restricted so that every implementation can
/// provide it cheaply. See README.md in this directory before changing it.
abstract interface class RawBox2D {
  // World lifecycle.

  /// Creates a world and returns its packed id.
  ///
  /// The parameters mirror `b2WorldDef`; implementations must start from the
  /// native defaults (`b2DefaultWorldDef`) and overwrite every field given
  /// here.
  int createWorld({
    required double gravityX,
    required double gravityY,
    required double restitutionThreshold,
    required double hitEventThreshold,
    required double contactHertz,
    required double contactDampingRatio,
    required double maxContactPushSpeed,
    required double maximumLinearSpeed,
    required bool enableSleep,
    required bool enableContinuous,
  });

  /// Destroys the world and everything in it.
  void destroyWorld(int worldId);

  /// Whether [worldId] refers to a live world.
  bool worldIsValid(int worldId);

  /// Advances the simulation by [timeStep] seconds using [subStepCount]
  /// sub-steps.
  void worldStep(int worldId, double timeStep, int subStepCount);

  /// Sets the world gravity vector.
  void worldSetGravity(int worldId, double x, double y);

  /// Returns the world gravity vector.
  (double, double) worldGetGravity(int worldId);

  /// Enables or disables sleeping on the world, waking all bodies when
  /// disabled.
  void worldEnableSleeping(int worldId, {required bool enabled});

  /// Whether body sleeping is enabled for the world.
  bool worldIsSleepingEnabled(int worldId);

  /// Enables or disables continuous collision detection.
  void worldEnableContinuous(int worldId, {required bool enabled});

  /// Whether continuous collision detection is enabled for the world.
  bool worldIsContinuousEnabled(int worldId);
}
