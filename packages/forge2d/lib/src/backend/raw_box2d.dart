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

  // Bodies.
  //
  // A body id crosses the seam as two ints: `index1`, and `wg`, which packs
  // the 16-bit world index and generation as `world0 | (generation << 16)`.
  // Shape, chain, and joint ids use the same encoding.

  /// Creates a body in the world and returns its id pair.
  ///
  /// The parameters mirror `b2BodyDef`; implementations start from
  /// `b2DefaultBodyDef` and overwrite every field given here. [type] is a
  /// `BodyType` index.
  (int, int) createBody(
    int worldId, {
    required int type,
    required double positionX,
    required double positionY,
    required double rotationCos,
    required double rotationSin,
    required double linearVelocityX,
    required double linearVelocityY,
    required double angularVelocity,
    required double linearDamping,
    required double angularDamping,
    required double gravityScale,
    required double sleepThreshold,
    required String? name,
    required bool enableSleep,
    required bool isAwake,
    required bool fixedRotation,
    required bool isBullet,
    required bool isEnabled,
    required bool allowFastRotation,
  });

  /// Destroys the body and all its shapes and joints.
  void destroyBody(int index1, int wg);

  /// Whether the id refers to a live body.
  bool bodyIsValid(int index1, int wg);

  /// Returns the body position.
  (double, double) bodyGetPosition(int index1, int wg);

  /// Returns the body rotation as a (cosine, sine) pair.
  (double, double) bodyGetRotation(int index1, int wg);

  /// Sets the body transform. Avoid moving bodies this way during regular
  /// simulation; create them at the right place instead.
  void bodySetTransform(
    int index1,
    int wg,
    double positionX,
    double positionY,
    double rotationCos,
    double rotationSin,
  );

  /// Returns the body's linear velocity, in meters per second.
  (double, double) bodyGetLinearVelocity(int index1, int wg);

  /// Sets the body's linear velocity, in meters per second.
  void bodySetLinearVelocity(int index1, int wg, double x, double y);

  /// Returns the body's angular velocity, in radians per second.
  double bodyGetAngularVelocity(int index1, int wg);

  /// Sets the body's angular velocity, in radians per second.
  void bodySetAngularVelocity(int index1, int wg, double value);

  /// Applies a force at a world point.
  void bodyApplyForce(
    int index1,
    int wg,
    double forceX,
    double forceY,
    double pointX,
    double pointY, {
    required bool wake,
  });

  /// Applies a force at the center of mass.
  void bodyApplyForceToCenter(
    int index1,
    int wg,
    double forceX,
    double forceY, {
    required bool wake,
  });

  /// Applies a torque.
  void bodyApplyTorque(int index1, int wg, double torque, {required bool wake});

  /// Applies an impulse at a world point.
  void bodyApplyLinearImpulse(
    int index1,
    int wg,
    double impulseX,
    double impulseY,
    double pointX,
    double pointY, {
    required bool wake,
  });

  /// Applies an impulse at the center of mass.
  void bodyApplyLinearImpulseToCenter(
    int index1,
    int wg,
    double impulseX,
    double impulseY, {
    required bool wake,
  });

  /// Applies an angular impulse.
  void bodyApplyAngularImpulse(
    int index1,
    int wg,
    double impulse, {
    required bool wake,
  });

  /// Returns the body mass in kilograms.
  double bodyGetMass(int index1, int wg);

  /// Returns the rotational inertia about the center of mass.
  double bodyGetRotationalInertia(int index1, int wg);

  /// Returns the local center of mass.
  (double, double) bodyGetLocalCenterOfMass(int index1, int wg);

  /// Returns the world center of mass.
  (double, double) bodyGetWorldCenterOfMass(int index1, int wg);

  /// Overrides the mass properties: mass, rotational inertia, local center.
  void bodySetMassData(
    int index1,
    int wg,
    double mass,
    double rotationalInertia,
    double centerX,
    double centerY,
  );

  /// Recomputes mass properties from the attached shapes.
  void bodyApplyMassFromShapes(int index1, int wg);

  /// Returns the `BodyType` index of the body.
  int bodyGetType(int index1, int wg);

  /// Sets the `BodyType` index of the body.
  void bodySetType(int index1, int wg, int type);

  /// Returns the body name.
  String bodyGetName(int index1, int wg);

  /// Sets the body name.
  void bodySetName(int index1, int wg, String? name);

  /// Whether the body is awake.
  bool bodyIsAwake(int index1, int wg);

  /// Wakes or puts the body to sleep.
  void bodySetAwake(int index1, int wg, {required bool awake});

  /// Whether the body may fall asleep.
  bool bodyIsSleepEnabled(int index1, int wg);

  /// Allows or prevents the body falling asleep.
  void bodyEnableSleep(int index1, int wg, {required bool enabled});

  /// Returns the sleep velocity threshold, in meters per second.
  double bodyGetSleepThreshold(int index1, int wg);

  /// Sets the sleep velocity threshold, in meters per second.
  void bodySetSleepThreshold(int index1, int wg, double value);

  /// Whether the body participates in the simulation.
  bool bodyIsEnabled(int index1, int wg);

  /// Removes the body from the simulation without destroying it.
  void bodyDisable(int index1, int wg);

  /// Puts a disabled body back into the simulation.
  void bodyEnable(int index1, int wg);

  /// Whether the body's rotation is fixed.
  bool bodyIsFixedRotation(int index1, int wg);

  /// Fixes or frees the body's rotation.
  void bodySetFixedRotation(int index1, int wg, {required bool flag});

  /// Whether the body is a bullet.
  bool bodyIsBullet(int index1, int wg);

  /// Marks the body as a bullet for continuous collision detection.
  void bodySetBullet(int index1, int wg, {required bool flag});

  /// Returns the gravity scale of the body.
  double bodyGetGravityScale(int index1, int wg);

  /// Sets the gravity scale of the body.
  void bodySetGravityScale(int index1, int wg, double scale);

  /// Returns the linear damping of the body.
  double bodyGetLinearDamping(int index1, int wg);

  /// Sets the linear damping of the body.
  void bodySetLinearDamping(int index1, int wg, double damping);

  /// Returns the angular damping of the body.
  double bodyGetAngularDamping(int index1, int wg);

  /// Sets the angular damping of the body.
  void bodySetAngularDamping(int index1, int wg, double damping);

  /// Converts a local point on the body to world coordinates.
  (double, double) bodyGetWorldPoint(int index1, int wg, double x, double y);

  /// Converts a world point to the body's local coordinates.
  (double, double) bodyGetLocalPoint(int index1, int wg, double x, double y);

  /// Returns the ids of the shapes attached to the body, as
  /// `[index1, wg, index1, wg, ...]`.
  List<int> bodyGetShapes(int index1, int wg);

  // Shapes.

  /// Creates a circle shape on the body. Returns the shape id pair.
  ///
  /// The def parameters mirror `b2ShapeDef`; implementations start from
  /// `b2DefaultShapeDef` and overwrite every field given here.
  (int, int) createCircleShape(
    int bodyIndex1,
    int bodyWg, {
    required double centerX,
    required double centerY,
    required double radius,
    required RawShapeDef def,
  });

  /// Creates a capsule shape on the body. Returns the shape id pair.
  (int, int) createCapsuleShape(
    int bodyIndex1,
    int bodyWg, {
    required double center1X,
    required double center1Y,
    required double center2X,
    required double center2Y,
    required double radius,
    required RawShapeDef def,
  });

  /// Creates a segment shape on the body. Returns the shape id pair.
  (int, int) createSegmentShape(
    int bodyIndex1,
    int bodyWg, {
    required double point1X,
    required double point1Y,
    required double point2X,
    required double point2Y,
    required RawShapeDef def,
  });

  /// Creates a box polygon shape on the body. Returns the shape id pair.
  ///
  /// The box has half-extents ([halfWidth], [halfHeight]), offset by
  /// ([centerX], [centerY]) and rotated by the given rotation.
  (int, int) createBoxShape(
    int bodyIndex1,
    int bodyWg, {
    required double halfWidth,
    required double halfHeight,
    required double centerX,
    required double centerY,
    required double rotationCos,
    required double rotationSin,
    required double radius,
    required RawShapeDef def,
  });

  /// Creates a polygon shape from the convex hull of the given points, with
  /// [points] packed as `[x0, y0, x1, y1, ...]`. Returns the shape id pair.
  ///
  /// Throws [ArgumentError] if the hull is degenerate.
  (int, int) createPolygonShape(
    int bodyIndex1,
    int bodyWg, {
    required List<double> points,
    required double radius,
    required RawShapeDef def,
  });

  /// Destroys the shape. When [updateBodyMass] is true the body's mass is
  /// recomputed.
  void destroyShape(int index1, int wg, {required bool updateBodyMass});

  /// Whether the id refers to a live shape.
  bool shapeIsValid(int index1, int wg);

  /// Returns the `ShapeType` index of the shape.
  int shapeGetType(int index1, int wg);

  /// Returns the id pair of the body owning the shape.
  (int, int) shapeGetBody(int index1, int wg);

  /// Whether the shape is a sensor.
  bool shapeIsSensor(int index1, int wg);

  /// Returns the density of the shape.
  double shapeGetDensity(int index1, int wg);

  /// Sets the density of the shape, optionally updating the body mass.
  void shapeSetDensity(
    int index1,
    int wg,
    double density, {
    required bool updateBodyMass,
  });

  /// Returns the friction of the shape.
  double shapeGetFriction(int index1, int wg);

  /// Sets the friction of the shape.
  void shapeSetFriction(int index1, int wg, double friction);

  /// Returns the restitution of the shape.
  double shapeGetRestitution(int index1, int wg);

  /// Sets the restitution of the shape.
  void shapeSetRestitution(int index1, int wg, double restitution);

  /// Returns the filter of the shape as (categoryBits, maskBits, groupIndex).
  (int, int, int) shapeGetFilter(int index1, int wg);

  /// Sets the filter of the shape.
  void shapeSetFilter(
    int index1,
    int wg,
    int categoryBits,
    int maskBits,
    int groupIndex,
  );

  /// Whether the shape reports sensor events.
  bool shapeAreSensorEventsEnabled(int index1, int wg);

  /// Enables or disables sensor events for the shape.
  void shapeEnableSensorEvents(int index1, int wg, {required bool enabled});

  /// Whether the shape reports contact events.
  bool shapeAreContactEventsEnabled(int index1, int wg);

  /// Enables or disables contact events for the shape.
  void shapeEnableContactEvents(int index1, int wg, {required bool enabled});

  /// Whether the shape reports hit events.
  bool shapeAreHitEventsEnabled(int index1, int wg);

  /// Enables or disables hit events for the shape.
  void shapeEnableHitEvents(int index1, int wg, {required bool enabled});

  /// Whether the shape reports pre-solve events.
  bool shapeArePreSolveEventsEnabled(int index1, int wg);

  /// Enables or disables pre-solve events for the shape.
  void shapeEnablePreSolveEvents(int index1, int wg, {required bool enabled});

  /// Whether the given world point is inside the shape.
  bool shapeTestPoint(int index1, int wg, double x, double y);

  /// Returns the current world axis-aligned bounding box of the shape as
  /// (lowerX, lowerY, upperX, upperY).
  (double, double, double, double) shapeGetAabb(int index1, int wg);

  // Chains.

  /// Creates a chain shape on the body, with [points] packed as
  /// `[x0, y0, x1, y1, ...]`. Returns the chain id pair.
  ///
  /// Each material is 6 values packed as
  /// `[friction, restitution, rollingResistance, tangentSpeed,
  /// userMaterialId, customColor]`; either one material, or one per segment.
  (int, int) createChain(
    int bodyIndex1,
    int bodyWg, {
    required List<double> points,
    required List<double> materials,
    required int categoryBits,
    required int maskBits,
    required int groupIndex,
    required bool isLoop,
    required bool enableSensorEvents,
  });

  /// Destroys the chain and its segments.
  void destroyChain(int index1, int wg);

  /// Whether the id refers to a live chain.
  bool chainIsValid(int index1, int wg);

  /// Sets the friction of all chain segments.
  void chainSetFriction(int index1, int wg, double friction);

  /// Returns the friction of the chain.
  double chainGetFriction(int index1, int wg);

  /// Sets the restitution of all chain segments.
  void chainSetRestitution(int index1, int wg, double restitution);

  /// Returns the restitution of the chain.
  double chainGetRestitution(int index1, int wg);

  /// Returns the ids of the segment shapes owned by the chain, as
  /// `[index1, wg, index1, wg, ...]`.
  List<int> chainGetSegments(int index1, int wg);
}

/// The `b2ShapeDef` fields flattened into a record so that the five shape
/// creation methods do not repeat a dozen parameters each.
///
/// Uses only core types, like the rest of the seam.
typedef RawShapeDef = ({
  double friction,
  double restitution,
  double rollingResistance,
  double tangentSpeed,
  int userMaterialId,
  int customColor,
  double density,
  int categoryBits,
  int maskBits,
  int groupIndex,
  bool isSensor,
  bool enableSensorEvents,
  bool enableContactEvents,
  bool enableHitEvents,
  bool enablePreSolveEvents,
  bool invokeContactCreation,
  bool updateBodyMass,
});
