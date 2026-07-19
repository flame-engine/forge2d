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
  // A body id crosses the seam as two ints: `index1`, and
  // `worldAndGeneration`, which packs
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
  void destroyBody(int index1, int worldAndGeneration);

  /// Whether the id refers to a live body.
  bool bodyIsValid(int index1, int worldAndGeneration);

  /// Returns the body position.
  (double, double) bodyGetPosition(int index1, int worldAndGeneration);

  /// Returns the body rotation as a (cosine, sine) pair.
  (double, double) bodyGetRotation(int index1, int worldAndGeneration);

  /// Sets the body transform. Avoid moving bodies this way during regular
  /// simulation; create them at the right place instead.
  void bodySetTransform(
    int index1,
    int worldAndGeneration,
    double positionX,
    double positionY,
    double rotationCos,
    double rotationSin,
  );

  /// Returns the body's linear velocity, in meters per second.
  (double, double) bodyGetLinearVelocity(int index1, int worldAndGeneration);

  /// Sets the body's linear velocity, in meters per second.
  void bodySetLinearVelocity(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  );

  /// Returns the body's angular velocity, in radians per second.
  double bodyGetAngularVelocity(int index1, int worldAndGeneration);

  /// Sets the body's angular velocity, in radians per second.
  void bodySetAngularVelocity(int index1, int worldAndGeneration, double value);

  /// Applies a force at a world point.
  void bodyApplyForce(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY,
    double pointX,
    double pointY, {
    required bool wake,
  });

  /// Applies a force at the center of mass.
  void bodyApplyForceToCenter(
    int index1,
    int worldAndGeneration,
    double forceX,
    double forceY, {
    required bool wake,
  });

  /// Applies a torque.
  void bodyApplyTorque(
    int index1,
    int worldAndGeneration,
    double torque, {
    required bool wake,
  });

  /// Applies an impulse at a world point.
  void bodyApplyLinearImpulse(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY,
    double pointX,
    double pointY, {
    required bool wake,
  });

  /// Applies an impulse at the center of mass.
  void bodyApplyLinearImpulseToCenter(
    int index1,
    int worldAndGeneration,
    double impulseX,
    double impulseY, {
    required bool wake,
  });

  /// Applies an angular impulse.
  void bodyApplyAngularImpulse(
    int index1,
    int worldAndGeneration,
    double impulse, {
    required bool wake,
  });

  /// Returns the body mass in kilograms.
  double bodyGetMass(int index1, int worldAndGeneration);

  /// Returns the rotational inertia about the center of mass.
  double bodyGetRotationalInertia(int index1, int worldAndGeneration);

  /// Returns the local center of mass.
  (double, double) bodyGetLocalCenterOfMass(int index1, int worldAndGeneration);

  /// Returns the world center of mass.
  (double, double) bodyGetWorldCenterOfMass(int index1, int worldAndGeneration);

  /// Overrides the mass properties: mass, rotational inertia, local center.
  void bodySetMassData(
    int index1,
    int worldAndGeneration,
    double mass,
    double rotationalInertia,
    double centerX,
    double centerY,
  );

  /// Recomputes mass properties from the attached shapes.
  void bodyApplyMassFromShapes(int index1, int worldAndGeneration);

  /// Returns the `BodyType` index of the body.
  int bodyGetType(int index1, int worldAndGeneration);

  /// Sets the `BodyType` index of the body.
  void bodySetType(int index1, int worldAndGeneration, int type);

  /// Returns the body name.
  String bodyGetName(int index1, int worldAndGeneration);

  /// Sets the body name.
  void bodySetName(int index1, int worldAndGeneration, String? name);

  /// Whether the body is awake.
  bool bodyIsAwake(int index1, int worldAndGeneration);

  /// Wakes or puts the body to sleep.
  void bodySetAwake(int index1, int worldAndGeneration, {required bool awake});

  /// Whether the body may fall asleep.
  bool bodyIsSleepEnabled(int index1, int worldAndGeneration);

  /// Allows or prevents the body falling asleep.
  void bodyEnableSleep(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the sleep velocity threshold, in meters per second.
  double bodyGetSleepThreshold(int index1, int worldAndGeneration);

  /// Sets the sleep velocity threshold, in meters per second.
  void bodySetSleepThreshold(int index1, int worldAndGeneration, double value);

  /// Whether the body participates in the simulation.
  bool bodyIsEnabled(int index1, int worldAndGeneration);

  /// Removes the body from the simulation without destroying it.
  void bodyDisable(int index1, int worldAndGeneration);

  /// Puts a disabled body back into the simulation.
  void bodyEnable(int index1, int worldAndGeneration);

  /// Whether the body's rotation is fixed.
  bool bodyIsFixedRotation(int index1, int worldAndGeneration);

  /// Fixes or frees the body's rotation.
  void bodySetFixedRotation(
    int index1,
    int worldAndGeneration, {
    required bool flag,
  });

  /// Whether the body is a bullet.
  bool bodyIsBullet(int index1, int worldAndGeneration);

  /// Marks the body as a bullet for continuous collision detection.
  void bodySetBullet(int index1, int worldAndGeneration, {required bool flag});

  /// Returns the gravity scale of the body.
  double bodyGetGravityScale(int index1, int worldAndGeneration);

  /// Sets the gravity scale of the body.
  void bodySetGravityScale(int index1, int worldAndGeneration, double scale);

  /// Returns the linear damping of the body.
  double bodyGetLinearDamping(int index1, int worldAndGeneration);

  /// Sets the linear damping of the body.
  void bodySetLinearDamping(int index1, int worldAndGeneration, double damping);

  /// Returns the angular damping of the body.
  double bodyGetAngularDamping(int index1, int worldAndGeneration);

  /// Sets the angular damping of the body.
  void bodySetAngularDamping(
    int index1,
    int worldAndGeneration,
    double damping,
  );

  /// Converts a local point on the body to world coordinates.
  (double, double) bodyGetWorldPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  );

  /// Converts a world point to the body's local coordinates.
  (double, double) bodyGetLocalPoint(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  );

  /// Returns the ids of the shapes attached to the body, as
  /// `[index1, worldAndGeneration, index1, worldAndGeneration, ...]`.
  List<int> bodyGetShapes(int index1, int worldAndGeneration);

  /// Returns the ids of the joints attached to the body, as
  /// `[index1, worldAndGeneration, index1, worldAndGeneration, ...]`.
  List<int> bodyGetJoints(int index1, int worldAndGeneration);

  // Shapes.

  /// Creates a circle shape on the body. Returns the shape id pair.
  ///
  /// The definition parameters mirror `b2ShapeDef`; implementations start from
  /// `b2DefaultShapeDef` and overwrite every field given here.
  (int, int) createCircleShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double centerX,
    required double centerY,
    required double radius,
    required RawShapeDef definition,
  });

  /// Creates a capsule shape on the body. Returns the shape id pair.
  (int, int) createCapsuleShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double center1X,
    required double center1Y,
    required double center2X,
    required double center2Y,
    required double radius,
    required RawShapeDef definition,
  });

  /// Creates a segment shape on the body. Returns the shape id pair.
  (int, int) createSegmentShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double point1X,
    required double point1Y,
    required double point2X,
    required double point2Y,
    required RawShapeDef definition,
  });

  /// Creates a box polygon shape on the body. Returns the shape id pair.
  ///
  /// The box has half-extents ([halfWidth], [halfHeight]), offset by
  /// ([centerX], [centerY]) and rotated by the given rotation.
  (int, int) createBoxShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required double halfWidth,
    required double halfHeight,
    required double centerX,
    required double centerY,
    required double rotationCos,
    required double rotationSin,
    required double radius,
    required RawShapeDef definition,
  });

  /// Creates a polygon shape from the convex hull of the given points, with
  /// [points] packed as `[x0, y0, x1, y1, ...]`. Returns the shape id pair.
  ///
  /// Throws [ArgumentError] if the hull is degenerate.
  (int, int) createPolygonShape(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required List<double> points,
    required double radius,
    required RawShapeDef definition,
  });

  /// Destroys the shape. When [updateBodyMass] is true the body's mass is
  /// recomputed.
  void destroyShape(
    int index1,
    int worldAndGeneration, {
    required bool updateBodyMass,
  });

  /// Whether the id refers to a live shape.
  bool shapeIsValid(int index1, int worldAndGeneration);

  /// Returns the `ShapeType` index of the shape.
  int shapeGetType(int index1, int worldAndGeneration);

  /// Returns the id pair of the body owning the shape.
  (int, int) shapeGetBody(int index1, int worldAndGeneration);

  /// Whether the shape is a sensor.
  bool shapeIsSensor(int index1, int worldAndGeneration);

  /// Returns the density of the shape.
  double shapeGetDensity(int index1, int worldAndGeneration);

  /// Sets the density of the shape, optionally updating the body mass.
  void shapeSetDensity(
    int index1,
    int worldAndGeneration,
    double density, {
    required bool updateBodyMass,
  });

  /// Returns the friction of the shape.
  double shapeGetFriction(int index1, int worldAndGeneration);

  /// Sets the friction of the shape.
  void shapeSetFriction(int index1, int worldAndGeneration, double friction);

  /// Returns the restitution of the shape.
  double shapeGetRestitution(int index1, int worldAndGeneration);

  /// Sets the restitution of the shape.
  void shapeSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  );

  /// Returns the filter of the shape as (categoryBits, maskBits, groupIndex).
  (int, int, int) shapeGetFilter(int index1, int worldAndGeneration);

  /// Sets the filter of the shape.
  void shapeSetFilter(
    int index1,
    int worldAndGeneration,
    int categoryBits,
    int maskBits,
    int groupIndex,
  );

  /// Whether the shape reports sensor events.
  bool shapeAreSensorEventsEnabled(int index1, int worldAndGeneration);

  /// Enables or disables sensor events for the shape.
  void shapeEnableSensorEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Whether the shape reports contact events.
  bool shapeAreContactEventsEnabled(int index1, int worldAndGeneration);

  /// Enables or disables contact events for the shape.
  void shapeEnableContactEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Whether the shape reports hit events.
  bool shapeAreHitEventsEnabled(int index1, int worldAndGeneration);

  /// Enables or disables hit events for the shape.
  void shapeEnableHitEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Whether the shape reports pre-solve events.
  bool shapeArePreSolveEventsEnabled(int index1, int worldAndGeneration);

  /// Enables or disables pre-solve events for the shape.
  void shapeEnablePreSolveEvents(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Whether the given world point is inside the shape.
  bool shapeTestPoint(int index1, int worldAndGeneration, double x, double y);

  /// Returns the current world axis-aligned bounding box of the shape as
  /// (lowerX, lowerY, upperX, upperY).
  (double, double, double, double) shapeGetAabb(
    int index1,
    int worldAndGeneration,
  );

  // Chains.

  /// Creates a chain shape on the body, with [points] packed as
  /// `[x0, y0, x1, y1, ...]`. Returns the chain id pair.
  ///
  /// Each material is 6 values packed as
  /// `[friction, restitution, rollingResistance, tangentSpeed,
  /// userMaterialId, customColor]`; either one material, or one per segment.
  (int, int) createChain(
    int bodyIndex1,
    int bodyWorldAndGeneration, {
    required List<double> points,
    required List<double> materials,
    required int categoryBits,
    required int maskBits,
    required int groupIndex,
    required bool isLoop,
    required bool enableSensorEvents,
  });

  /// Destroys the chain and its segments.
  void destroyChain(int index1, int worldAndGeneration);

  /// Whether the id refers to a live chain.
  bool chainIsValid(int index1, int worldAndGeneration);

  /// Sets the friction of all chain segments.
  void chainSetFriction(int index1, int worldAndGeneration, double friction);

  /// Returns the friction of the chain.
  double chainGetFriction(int index1, int worldAndGeneration);

  /// Sets the restitution of all chain segments.
  void chainSetRestitution(
    int index1,
    int worldAndGeneration,
    double restitution,
  );

  /// Returns the restitution of the chain.
  double chainGetRestitution(int index1, int worldAndGeneration);

  /// Returns the ids of the segment shapes owned by the chain, as
  /// `[index1, worldAndGeneration, index1, worldAndGeneration, ...]`.
  List<int> chainGetSegments(int index1, int worldAndGeneration);

  // Joints.
  //
  // The create methods mirror the `b2*JointDef` structs; implementations
  // start from the native defaults and overwrite every field given here.
  // [bodyA] and [bodyB] are (index1, worldAndGeneration) id pairs.

  /// Creates a distance joint. Returns the joint id pair.
  (int, int) createDistanceJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double length,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double minLength,
    required double maxLength,
    required bool enableMotor,
    required double maxMotorForce,
    required double motorSpeed,
    required bool collideConnected,
  });

  /// Creates a filter joint, which only disables collision between the two
  /// bodies. Returns the joint id pair.
  (int, int) createFilterJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
  });

  /// Creates a motor joint. Returns the joint id pair.
  (int, int) createMotorJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) linearOffset,
    required double angularOffset,
    required double maxForce,
    required double maxTorque,
    required double correctionFactor,
    required bool collideConnected,
  });

  /// Creates a mouse joint. Returns the joint id pair.
  (int, int) createMouseJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) target,
    required double hertz,
    required double dampingRatio,
    required double maxForce,
    required bool collideConnected,
  });

  /// Creates a prismatic joint. Returns the joint id pair.
  (int, int) createPrismaticJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required (double, double) localAxisA,
    required double referenceAngle,
    required double targetTranslation,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerTranslation,
    required double upperTranslation,
    required bool enableMotor,
    required double maxMotorForce,
    required double motorSpeed,
    required bool collideConnected,
  });

  /// Creates a revolute joint. Returns the joint id pair.
  (int, int) createRevoluteJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double referenceAngle,
    required double targetAngle,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerAngle,
    required double upperAngle,
    required bool enableMotor,
    required double maxMotorTorque,
    required double motorSpeed,
    required double drawSize,
    required bool collideConnected,
  });

  /// Creates a weld joint. Returns the joint id pair.
  (int, int) createWeldJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required double referenceAngle,
    required double linearHertz,
    required double angularHertz,
    required double linearDampingRatio,
    required double angularDampingRatio,
    required bool collideConnected,
  });

  /// Creates a wheel joint. Returns the joint id pair.
  (int, int) createWheelJoint(
    int worldId, {
    required (int, int) bodyA,
    required (int, int) bodyB,
    required (double, double) localAnchorA,
    required (double, double) localAnchorB,
    required (double, double) localAxisA,
    required bool enableSpring,
    required double hertz,
    required double dampingRatio,
    required bool enableLimit,
    required double lowerTranslation,
    required double upperTranslation,
    required bool enableMotor,
    required double maxMotorTorque,
    required double motorSpeed,
    required bool collideConnected,
  });

  /// Destroys the joint.
  void destroyJoint(int index1, int worldAndGeneration);

  /// Whether the id refers to a live joint.
  bool jointIsValid(int index1, int worldAndGeneration);

  /// Returns the `JointType` index of the joint.
  int jointGetType(int index1, int worldAndGeneration);

  /// Returns the id pair of the first attached body.
  (int, int) jointGetBodyA(int index1, int worldAndGeneration);

  /// Returns the id pair of the second attached body.
  (int, int) jointGetBodyB(int index1, int worldAndGeneration);

  /// Returns the local anchor on the first body.
  (double, double) jointGetLocalAnchorA(int index1, int worldAndGeneration);

  /// Returns the local anchor on the second body.
  (double, double) jointGetLocalAnchorB(int index1, int worldAndGeneration);

  /// Whether the connected bodies can collide.
  bool jointGetCollideConnected(int index1, int worldAndGeneration);

  /// Sets whether the connected bodies can collide.
  void jointSetCollideConnected(
    int index1,
    int worldAndGeneration, {
    required bool value,
  });

  /// Wakes the connected bodies.
  void jointWakeBodies(int index1, int worldAndGeneration);

  /// Returns the constraint force on the joint, in newtons.
  (double, double) jointGetConstraintForce(int index1, int worldAndGeneration);

  /// Returns the constraint torque on the joint, in newton-meters.
  double jointGetConstraintTorque(int index1, int worldAndGeneration);

  // Distance joint.

  /// Returns the rest length.
  double distanceJointGetLength(int index1, int worldAndGeneration);

  /// Sets the rest length.
  void distanceJointSetLength(
    int index1,
    int worldAndGeneration,
    double length,
  );

  /// Returns the current distance between the anchors.
  double distanceJointGetCurrentLength(int index1, int worldAndGeneration);

  /// Whether the spring is enabled.
  bool distanceJointIsSpringEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the spring.
  void distanceJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the spring stiffness in hertz.
  double distanceJointGetSpringHertz(int index1, int worldAndGeneration);

  /// Sets the spring stiffness in hertz.
  void distanceJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the spring damping ratio.
  double distanceJointGetSpringDampingRatio(int index1, int worldAndGeneration);

  /// Sets the spring damping ratio.
  void distanceJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Whether the length limit is enabled.
  bool distanceJointIsLimitEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the length limit.
  void distanceJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the minimum length.
  double distanceJointGetMinLength(int index1, int worldAndGeneration);

  /// Returns the maximum length.
  double distanceJointGetMaxLength(int index1, int worldAndGeneration);

  /// Sets the length limit range.
  void distanceJointSetLengthRange(
    int index1,
    int worldAndGeneration,
    double minLength,
    double maxLength,
  );

  /// Whether the motor is enabled.
  bool distanceJointIsMotorEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the motor.
  void distanceJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the motor speed, in meters per second.
  double distanceJointGetMotorSpeed(int index1, int worldAndGeneration);

  /// Sets the motor speed, in meters per second.
  void distanceJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  );

  /// Returns the maximum motor force, in newtons.
  double distanceJointGetMaxMotorForce(int index1, int worldAndGeneration);

  /// Sets the maximum motor force, in newtons.
  void distanceJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  );

  /// Returns the current motor force, in newtons.
  double distanceJointGetMotorForce(int index1, int worldAndGeneration);

  // Motor joint.

  /// Returns the target linear offset in the frame of body A.
  (double, double) motorJointGetLinearOffset(
    int index1,
    int worldAndGeneration,
  );

  /// Sets the target linear offset in the frame of body A.
  void motorJointSetLinearOffset(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  );

  /// Returns the target angular offset.
  double motorJointGetAngularOffset(int index1, int worldAndGeneration);

  /// Sets the target angular offset.
  void motorJointSetAngularOffset(
    int index1,
    int worldAndGeneration,
    double offset,
  );

  /// Returns the maximum force, in newtons.
  double motorJointGetMaxForce(int index1, int worldAndGeneration);

  /// Sets the maximum force, in newtons.
  void motorJointSetMaxForce(int index1, int worldAndGeneration, double force);

  /// Returns the maximum torque, in newton-meters.
  double motorJointGetMaxTorque(int index1, int worldAndGeneration);

  /// Sets the maximum torque, in newton-meters.
  void motorJointSetMaxTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  );

  /// Returns the position correction factor, in `[0, 1]`.
  double motorJointGetCorrectionFactor(int index1, int worldAndGeneration);

  /// Sets the position correction factor, in `[0, 1]`.
  void motorJointSetCorrectionFactor(
    int index1,
    int worldAndGeneration,
    double factor,
  );

  // Mouse joint.

  /// Returns the target point.
  (double, double) mouseJointGetTarget(int index1, int worldAndGeneration);

  /// Sets the target point.
  void mouseJointSetTarget(
    int index1,
    int worldAndGeneration,
    double x,
    double y,
  );

  /// Returns the spring stiffness in hertz.
  double mouseJointGetSpringHertz(int index1, int worldAndGeneration);

  /// Sets the spring stiffness in hertz.
  void mouseJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the spring damping ratio.
  double mouseJointGetSpringDampingRatio(int index1, int worldAndGeneration);

  /// Sets the spring damping ratio.
  void mouseJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Returns the maximum force, in newtons.
  double mouseJointGetMaxForce(int index1, int worldAndGeneration);

  /// Sets the maximum force, in newtons.
  void mouseJointSetMaxForce(int index1, int worldAndGeneration, double force);

  // Prismatic joint.

  /// Whether the spring is enabled.
  bool prismaticJointIsSpringEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the spring.
  void prismaticJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the spring stiffness in hertz.
  double prismaticJointGetSpringHertz(int index1, int worldAndGeneration);

  /// Sets the spring stiffness in hertz.
  void prismaticJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the spring damping ratio.
  double prismaticJointGetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
  );

  /// Sets the spring damping ratio.
  void prismaticJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Returns the target translation, in meters.
  double prismaticJointGetTargetTranslation(int index1, int worldAndGeneration);

  /// Sets the target translation, in meters.
  void prismaticJointSetTargetTranslation(
    int index1,
    int worldAndGeneration,
    double value,
  );

  /// Whether the translation limit is enabled.
  bool prismaticJointIsLimitEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the translation limit.
  void prismaticJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the lower translation limit.
  double prismaticJointGetLowerLimit(int index1, int worldAndGeneration);

  /// Returns the upper translation limit.
  double prismaticJointGetUpperLimit(int index1, int worldAndGeneration);

  /// Sets the translation limits.
  void prismaticJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  );

  /// Whether the motor is enabled.
  bool prismaticJointIsMotorEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the motor.
  void prismaticJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the motor speed, in meters per second.
  double prismaticJointGetMotorSpeed(int index1, int worldAndGeneration);

  /// Sets the motor speed, in meters per second.
  void prismaticJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  );

  /// Returns the maximum motor force, in newtons.
  double prismaticJointGetMaxMotorForce(int index1, int worldAndGeneration);

  /// Sets the maximum motor force, in newtons.
  void prismaticJointSetMaxMotorForce(
    int index1,
    int worldAndGeneration,
    double force,
  );

  /// Returns the current motor force, in newtons.
  double prismaticJointGetMotorForce(int index1, int worldAndGeneration);

  /// Returns the current translation, in meters.
  double prismaticJointGetTranslation(int index1, int worldAndGeneration);

  /// Returns the current translation speed, in meters per second.
  double prismaticJointGetSpeed(int index1, int worldAndGeneration);

  // Revolute joint.

  /// Whether the spring is enabled.
  bool revoluteJointIsSpringEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the spring.
  void revoluteJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the spring stiffness in hertz.
  double revoluteJointGetSpringHertz(int index1, int worldAndGeneration);

  /// Sets the spring stiffness in hertz.
  void revoluteJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the spring damping ratio.
  double revoluteJointGetSpringDampingRatio(int index1, int worldAndGeneration);

  /// Sets the spring damping ratio.
  void revoluteJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Returns the target angle, in radians.
  double revoluteJointGetTargetAngle(int index1, int worldAndGeneration);

  /// Sets the target angle, in radians.
  void revoluteJointSetTargetAngle(
    int index1,
    int worldAndGeneration,
    double angle,
  );

  /// Returns the current joint angle, in radians.
  double revoluteJointGetAngle(int index1, int worldAndGeneration);

  /// Whether the angle limit is enabled.
  bool revoluteJointIsLimitEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the angle limit.
  void revoluteJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the lower angle limit, in radians.
  double revoluteJointGetLowerLimit(int index1, int worldAndGeneration);

  /// Returns the upper angle limit, in radians.
  double revoluteJointGetUpperLimit(int index1, int worldAndGeneration);

  /// Sets the angle limits, in radians.
  void revoluteJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  );

  /// Whether the motor is enabled.
  bool revoluteJointIsMotorEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the motor.
  void revoluteJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the motor speed, in radians per second.
  double revoluteJointGetMotorSpeed(int index1, int worldAndGeneration);

  /// Sets the motor speed, in radians per second.
  void revoluteJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  );

  /// Returns the maximum motor torque, in newton-meters.
  double revoluteJointGetMaxMotorTorque(int index1, int worldAndGeneration);

  /// Sets the maximum motor torque, in newton-meters.
  void revoluteJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  );

  /// Returns the current motor torque, in newton-meters.
  double revoluteJointGetMotorTorque(int index1, int worldAndGeneration);

  // Weld joint.

  /// Returns the linear stiffness in hertz.
  double weldJointGetLinearHertz(int index1, int worldAndGeneration);

  /// Sets the linear stiffness in hertz (zero for rigid).
  void weldJointSetLinearHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the angular stiffness in hertz.
  double weldJointGetAngularHertz(int index1, int worldAndGeneration);

  /// Sets the angular stiffness in hertz (zero for rigid).
  void weldJointSetAngularHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the linear damping ratio.
  double weldJointGetLinearDampingRatio(int index1, int worldAndGeneration);

  /// Sets the linear damping ratio.
  void weldJointSetLinearDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Returns the angular damping ratio.
  double weldJointGetAngularDampingRatio(int index1, int worldAndGeneration);

  /// Sets the angular damping ratio.
  void weldJointSetAngularDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  // Wheel joint.

  /// Whether the spring is enabled.
  bool wheelJointIsSpringEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the spring.
  void wheelJointEnableSpring(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the spring stiffness in hertz.
  double wheelJointGetSpringHertz(int index1, int worldAndGeneration);

  /// Sets the spring stiffness in hertz.
  void wheelJointSetSpringHertz(
    int index1,
    int worldAndGeneration,
    double hertz,
  );

  /// Returns the spring damping ratio.
  double wheelJointGetSpringDampingRatio(int index1, int worldAndGeneration);

  /// Sets the spring damping ratio.
  void wheelJointSetSpringDampingRatio(
    int index1,
    int worldAndGeneration,
    double ratio,
  );

  /// Whether the translation limit is enabled.
  bool wheelJointIsLimitEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the translation limit.
  void wheelJointEnableLimit(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the lower translation limit.
  double wheelJointGetLowerLimit(int index1, int worldAndGeneration);

  /// Returns the upper translation limit.
  double wheelJointGetUpperLimit(int index1, int worldAndGeneration);

  /// Sets the translation limits.
  void wheelJointSetLimits(
    int index1,
    int worldAndGeneration,
    double lower,
    double upper,
  );

  /// Whether the motor is enabled.
  bool wheelJointIsMotorEnabled(int index1, int worldAndGeneration);

  /// Enables or disables the motor.
  void wheelJointEnableMotor(
    int index1,
    int worldAndGeneration, {
    required bool enabled,
  });

  /// Returns the motor speed, in radians per second.
  double wheelJointGetMotorSpeed(int index1, int worldAndGeneration);

  /// Sets the motor speed, in radians per second.
  void wheelJointSetMotorSpeed(
    int index1,
    int worldAndGeneration,
    double speed,
  );

  /// Returns the maximum motor torque, in newton-meters.
  double wheelJointGetMaxMotorTorque(int index1, int worldAndGeneration);

  /// Sets the maximum motor torque, in newton-meters.
  void wheelJointSetMaxMotorTorque(
    int index1,
    int worldAndGeneration,
    double torque,
  );

  /// Returns the current motor torque, in newton-meters.
  double wheelJointGetMotorTorque(int index1, int worldAndGeneration);

  // Events (polled after each step). Implementations copy the native event
  // arrays into Dart lists, so the returned data stays usable after later
  // steps.

  /// Returns the contact events of the last step.
  RawContactEvents worldGetContactEvents(int worldId);

  /// Returns the sensor events of the last step.
  RawSensorEvents worldGetSensorEvents(int worldId);

  /// Returns the body move events of the last step.
  List<RawBodyMoveEvent> worldGetBodyEvents(int worldId);

  // Queries.

  /// Casts a ray and returns the closest hit, or null when nothing is hit.
  RawRayHit? worldCastRayClosest(
    int worldId,
    double originX,
    double originY,
    double translationX,
    double translationY,
    int categoryBits,
    int maskBits,
  );

  /// Casts a ray, invoking [callback] for every candidate hit.
  ///
  /// The callback controls the continuation of the cast with its return
  /// value: -1 ignores the hit, 0 terminates, the fraction clips the ray,
  /// and 1 continues unclipped.
  void worldCastRay(
    int worldId,
    double originX,
    double originY,
    double translationX,
    double translationY,
    int categoryBits,
    int maskBits,
    double Function(RawRayHit hit) callback,
  );

  /// Returns the ids of all shapes whose bounding boxes overlap the given
  /// box, as `[index1, worldAndGeneration, index1, worldAndGeneration, ...]`.
  List<int> worldOverlapAabb(
    int worldId,
    double lowerX,
    double lowerY,
    double upperX,
    double upperY,
    int categoryBits,
    int maskBits,
  );

  /// Applies a radial explosion impulse.
  void worldExplode(
    int worldId, {
    required int maskBits,
    required double positionX,
    required double positionY,
    required double radius,
    required double falloff,
    required double impulsePerLength,
  });

  // Simulation callbacks. Single-threaded worlds only.

  /// Sets or clears the custom collision filter callback.
  ///
  /// The callback receives both shape id pairs and returns whether the
  /// shapes should collide.
  void worldSetCustomFilterCallback(
    int worldId,
    bool Function(
      int shapeAIndex1,
      int shapeAWorldAndGeneration,
      int shapeBIndex1,
      int shapeBWorldAndGeneration,
    )?
    callback,
  );

  /// Sets or clears the pre-solve callback, invoked during the step after
  /// contact points are found but before the solver runs.
  ///
  /// The callback returns whether the contact should be solved.
  void worldSetPreSolveCallback(
    int worldId,
    bool Function(
      int shapeAIndex1,
      int shapeAWorldAndGeneration,
      int shapeBIndex1,
      int shapeBWorldAndGeneration,
      double normalX,
      double normalY,
    )?
    callback,
  );
}

/// A contact manifold point: the contact position and the separation
/// (negative when overlapping).
typedef RawManifoldPoint = ({double x, double y, double separation});

/// A begin-touch contact event with its initial manifold.
typedef RawContactBeginEvent = ({
  int shapeAIndex1,
  int shapeAWorldAndGeneration,
  int shapeBIndex1,
  int shapeBWorldAndGeneration,
  double normalX,
  double normalY,
  List<RawManifoldPoint> points,
});

/// An end-touch contact event. The shapes may already be destroyed.
typedef RawContactEndEvent = ({
  int shapeAIndex1,
  int shapeAWorldAndGeneration,
  int shapeBIndex1,
  int shapeBWorldAndGeneration,
});

/// A contact hit event, reported for impacts above the world's hit event
/// threshold.
typedef RawContactHitEvent = ({
  int shapeAIndex1,
  int shapeAWorldAndGeneration,
  int shapeBIndex1,
  int shapeBWorldAndGeneration,
  double pointX,
  double pointY,
  double normalX,
  double normalY,
  double approachSpeed,
});

/// The contact events of one step.
typedef RawContactEvents = ({
  List<RawContactBeginEvent> begin,
  List<RawContactEndEvent> end,
  List<RawContactHitEvent> hit,
});

/// A sensor overlap event. On end events the shapes may be destroyed.
typedef RawSensorEvent = ({
  int sensorIndex1,
  int sensorWorldAndGeneration,
  int visitorIndex1,
  int visitorWorldAndGeneration,
});

/// The sensor events of one step.
typedef RawSensorEvents = ({
  List<RawSensorEvent> begin,
  List<RawSensorEvent> end,
});

/// A body move event: the body's new transform, and whether it fell asleep.
typedef RawBodyMoveEvent = ({
  int bodyIndex1,
  int bodyWorldAndGeneration,
  double x,
  double y,
  double rotationCos,
  double rotationSin,
  bool fellAsleep,
});

/// A ray cast hit.
typedef RawRayHit = ({
  int shapeIndex1,
  int shapeWorldAndGeneration,
  double pointX,
  double pointY,
  double normalX,
  double normalY,
  double fraction,
});

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
