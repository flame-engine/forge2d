import 'package:forge2d/src/api/chain.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/geometry.dart';
import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/api/math.dart';
import 'package:forge2d/src/api/shape.dart';
import 'package:forge2d/src/api/world.dart';
import 'package:forge2d/src/backend/raw_box2d.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The mass properties of a body.
///
/// Mirrors the native `b2MassData` type.
class MassData {
  /// Creates mass data.
  MassData({
    required this.mass,
    required Vector2 center,
    required this.rotationalInertia,
  }) : center = center.clone();

  /// The mass, in kilograms.
  final double mass;

  /// The center of mass relative to the body origin.
  final Vector2 center;

  /// The rotational inertia about the center of mass, in kg*m^2.
  final double rotationalInertia;
}

/// A rigid body.
///
/// Wraps a native `b2BodyId`. A [Body] instance is a cheap value-like
/// handle: two handles to the same body compare equal, and holding one does
/// not keep the body alive. Destroy bodies explicitly with [destroy], or
/// destroy the whole [World].
@immutable
class Body {
  /// Wraps an existing native body id. Internal to forge2d.
  @internal
  const Body.internal(this.world, this.index1, this.worldAndGeneration);

  /// The world this body belongs to.
  final World world;

  /// The first half of the packed native body id.
  @internal
  final int index1;

  /// The second half of the packed native body id.
  @internal
  final int worldAndGeneration;

  /// Whether this body has not been destroyed.
  bool get isValid => rawBox2D.bodyIsValid(index1, worldAndGeneration);

  /// The world position of the body origin.
  Vector2 get position {
    final (x, y) = rawBox2D.bodyGetPosition(index1, worldAndGeneration);
    return Vector2(x, y);
  }

  /// The world rotation of the body.
  Rot get rotation {
    final (cos, sin) = rawBox2D.bodyGetRotation(index1, worldAndGeneration);
    return Rot(cos, sin);
  }

  /// The world rotation angle of the body in radians.
  double get angle => rotation.angle;

  /// The body transform.
  Transform get transform => Transform(position, rotation);

  /// Teleports the body to a new position and rotation.
  ///
  /// Avoid using this during regular simulation; create bodies at the right
  /// place instead, and move them with velocities and forces.
  void setTransform(Vector2 position, Rot rotation) {
    rawBox2D.bodySetTransform(
      index1,
      worldAndGeneration,
      position.x,
      position.y,
      rotation.cos,
      rotation.sin,
    );
  }

  /// The linear velocity of the body origin, in meters per second.
  Vector2 get linearVelocity {
    final (x, y) = rawBox2D.bodyGetLinearVelocity(index1, worldAndGeneration);
    return Vector2(x, y);
  }

  set linearVelocity(Vector2 value) => rawBox2D.bodySetLinearVelocity(
    index1,
    worldAndGeneration,
    value.x,
    value.y,
  );

  /// The angular velocity of the body, in radians per second.
  double get angularVelocity =>
      rawBox2D.bodyGetAngularVelocity(index1, worldAndGeneration);

  set angularVelocity(double value) =>
      rawBox2D.bodySetAngularVelocity(index1, worldAndGeneration, value);

  /// Applies [force] at world [point], or at the center of mass when
  /// [point] is omitted.
  ///
  /// Forces are applied continuously over the time step. When [wake] is
  /// false and the body is sleeping, the force is ignored.
  void applyForce(Vector2 force, {Vector2? point, bool wake = true}) {
    if (point == null) {
      rawBox2D.bodyApplyForceToCenter(
        index1,
        worldAndGeneration,
        force.x,
        force.y,
        wake: wake,
      );
    } else {
      rawBox2D.bodyApplyForce(
        index1,
        worldAndGeneration,
        force.x,
        force.y,
        point.x,
        point.y,
        wake: wake,
      );
    }
  }

  /// Applies a torque, affecting the angular velocity over the time step.
  void applyTorque(double torque, {bool wake = true}) =>
      rawBox2D.bodyApplyTorque(index1, worldAndGeneration, torque, wake: wake);

  /// Applies [impulse] at world [point], or at the center of mass when
  /// [point] is omitted, immediately changing the linear velocity.
  void applyLinearImpulse(Vector2 impulse, {Vector2? point, bool wake = true}) {
    if (point == null) {
      rawBox2D.bodyApplyLinearImpulseToCenter(
        index1,
        worldAndGeneration,
        impulse.x,
        impulse.y,
        wake: wake,
      );
    } else {
      rawBox2D.bodyApplyLinearImpulse(
        index1,
        worldAndGeneration,
        impulse.x,
        impulse.y,
        point.x,
        point.y,
        wake: wake,
      );
    }
  }

  /// Applies an angular impulse, immediately changing the angular velocity.
  void applyAngularImpulse(double impulse, {bool wake = true}) => rawBox2D
      .bodyApplyAngularImpulse(index1, worldAndGeneration, impulse, wake: wake);

  /// The mass of the body, in kilograms.
  double get mass => rawBox2D.bodyGetMass(index1, worldAndGeneration);

  /// The rotational inertia about the center of mass, in kg*m^2.
  double get rotationalInertia =>
      rawBox2D.bodyGetRotationalInertia(index1, worldAndGeneration);

  /// The center of mass relative to the body origin.
  Vector2 get localCenterOfMass {
    final (x, y) = rawBox2D.bodyGetLocalCenterOfMass(
      index1,
      worldAndGeneration,
    );
    return Vector2(x, y);
  }

  /// The center of mass in world coordinates.
  Vector2 get worldCenterOfMass {
    final (x, y) = rawBox2D.bodyGetWorldCenterOfMass(
      index1,
      worldAndGeneration,
    );
    return Vector2(x, y);
  }

  /// The mass properties of the body.
  MassData get massData => MassData(
    mass: mass,
    center: localCenterOfMass,
    rotationalInertia: rotationalInertia,
  );

  /// Overrides the mass properties computed from the shapes.
  set massData(MassData value) => rawBox2D.bodySetMassData(
    index1,
    worldAndGeneration,
    value.mass,
    value.rotationalInertia,
    value.center.x,
    value.center.y,
  );

  /// Recomputes the mass properties from the attached shapes, discarding any
  /// override set through [massData].
  void applyMassFromShapes() =>
      rawBox2D.bodyApplyMassFromShapes(index1, worldAndGeneration);

  /// The body type: static, kinematic, or dynamic.
  ///
  /// Changing the type is expensive: it wakes touching bodies and recreates
  /// contact data.
  BodyType get type =>
      BodyType.values[rawBox2D.bodyGetType(index1, worldAndGeneration)];

  set type(BodyType value) =>
      rawBox2D.bodySetType(index1, worldAndGeneration, value.index);

  /// The body name, for debugging. Empty when unset.
  String get name => rawBox2D.bodyGetName(index1, worldAndGeneration);

  set name(String? value) =>
      rawBox2D.bodySetName(index1, worldAndGeneration, value);

  /// The user data associated with the body.
  Object? get userData => world.bodyUserData[(index1, worldAndGeneration)];

  set userData(Object? value) {
    if (value == null) {
      world.bodyUserData.remove((index1, worldAndGeneration));
    } else {
      world.bodyUserData[(index1, worldAndGeneration)] = value;
    }
  }

  /// Whether the body is awake and being simulated.
  bool get isAwake => rawBox2D.bodyIsAwake(index1, worldAndGeneration);

  set isAwake(bool value) =>
      rawBox2D.bodySetAwake(index1, worldAndGeneration, awake: value);

  /// Whether the body may fall asleep when it comes to rest.
  bool get sleepEnabled =>
      rawBox2D.bodyIsSleepEnabled(index1, worldAndGeneration);

  set sleepEnabled(bool value) =>
      rawBox2D.bodyEnableSleep(index1, worldAndGeneration, enabled: value);

  /// The speed below which the body may fall asleep, in meters per second.
  double get sleepThreshold =>
      rawBox2D.bodyGetSleepThreshold(index1, worldAndGeneration);

  set sleepThreshold(double value) =>
      rawBox2D.bodySetSleepThreshold(index1, worldAndGeneration, value);

  /// Whether the body participates in the simulation.
  ///
  /// Disabling a body is expensive: it removes the body from the broad
  /// phase and destroys its contacts.
  bool get isEnabled => rawBox2D.bodyIsEnabled(index1, worldAndGeneration);

  set isEnabled(bool value) {
    if (value) {
      rawBox2D.bodyEnable(index1, worldAndGeneration);
    } else {
      rawBox2D.bodyDisable(index1, worldAndGeneration);
    }
  }

  /// Whether the body rotation is fixed.
  bool get fixedRotation =>
      rawBox2D.bodyIsFixedRotation(index1, worldAndGeneration);

  set fixedRotation(bool value) =>
      rawBox2D.bodySetFixedRotation(index1, worldAndGeneration, flag: value);

  /// Whether the body uses continuous collision detection against static
  /// and kinematic bodies.
  bool get isBullet => rawBox2D.bodyIsBullet(index1, worldAndGeneration);

  set isBullet(bool value) =>
      rawBox2D.bodySetBullet(index1, worldAndGeneration, flag: value);

  /// The scale applied to world gravity for this body.
  double get gravityScale =>
      rawBox2D.bodyGetGravityScale(index1, worldAndGeneration);

  set gravityScale(double value) =>
      rawBox2D.bodySetGravityScale(index1, worldAndGeneration, value);

  /// Damping applied to the linear velocity.
  double get linearDamping =>
      rawBox2D.bodyGetLinearDamping(index1, worldAndGeneration);

  set linearDamping(double value) =>
      rawBox2D.bodySetLinearDamping(index1, worldAndGeneration, value);

  /// Damping applied to the angular velocity.
  double get angularDamping =>
      rawBox2D.bodyGetAngularDamping(index1, worldAndGeneration);

  set angularDamping(double value) =>
      rawBox2D.bodySetAngularDamping(index1, worldAndGeneration, value);

  /// Converts a point on the body to world coordinates.
  Vector2 worldPoint(Vector2 localPoint) {
    final (x, y) = rawBox2D.bodyGetWorldPoint(
      index1,
      worldAndGeneration,
      localPoint.x,
      localPoint.y,
    );
    return Vector2(x, y);
  }

  /// Converts a world point to the body's local coordinates.
  Vector2 localPoint(Vector2 worldPoint) {
    final (x, y) = rawBox2D.bodyGetLocalPoint(
      index1,
      worldAndGeneration,
      worldPoint.x,
      worldPoint.y,
    );
    return Vector2(x, y);
  }

  /// Creates a shape with the given [geometry] and attaches it to the body.
  ///
  /// Cannot be called while the world is stepping.
  Shape createShape(ShapeGeometry geometry, [ShapeDef? definition]) {
    world.checkCanMutate('create a shape');
    final shapeDefinition = definition ?? ShapeDef();
    assert(
      shapeDefinition.density >= 0,
      'ShapeDef.density must not be negative',
    );
    assert(
      switch (geometry) {
        Circle(:final radius) => radius > 0 && radius.isFinite,
        Capsule(:final radius) => radius > 0 && radius.isFinite,
        _ => true,
      },
      'The geometry radius must be positive and finite',
    );
    final rawDefinition = _rawShapeDef(shapeDefinition);
    final (shapeIndex1, shapeWorldAndGeneration) = switch (geometry) {
      Circle(:final center, :final radius) => rawBox2D.createCircleShape(
        index1,
        worldAndGeneration,
        centerX: center.x,
        centerY: center.y,
        radius: radius,
        definition: rawDefinition,
      ),
      Capsule(:final center1, :final center2, :final radius) =>
        rawBox2D.createCapsuleShape(
          index1,
          worldAndGeneration,
          center1X: center1.x,
          center1Y: center1.y,
          center2X: center2.x,
          center2Y: center2.y,
          radius: radius,
          definition: rawDefinition,
        ),
      Segment(:final point1, :final point2) => rawBox2D.createSegmentShape(
        index1,
        worldAndGeneration,
        point1X: point1.x,
        point1Y: point1.y,
        point2X: point2.x,
        point2Y: point2.y,
        definition: rawDefinition,
      ),
      Polygon(points: final points?, :final radius) =>
        rawBox2D.createPolygonShape(
          index1,
          worldAndGeneration,
          points: [
            for (final point in points) ...[point.x, point.y],
          ],
          radius: radius,
          definition: rawDefinition,
        ),
      Polygon(
        :final halfWidth?,
        :final halfHeight?,
        :final center,
        :final rotation,
        :final radius,
      ) =>
        rawBox2D.createBoxShape(
          index1,
          worldAndGeneration,
          halfWidth: halfWidth,
          halfHeight: halfHeight,
          centerX: center?.x ?? 0,
          centerY: center?.y ?? 0,
          rotationCos: rotation == null ? 1 : Rot.fromAngle(rotation).cos,
          rotationSin: rotation == null ? 0 : Rot.fromAngle(rotation).sin,
          radius: radius,
          definition: rawDefinition,
        ),
      Polygon() => throw StateError('Unreachable polygon configuration'),
    };
    if (shapeDefinition.userData != null) {
      world.shapeUserData[(shapeIndex1, shapeWorldAndGeneration)] =
          shapeDefinition.userData;
    }
    return Shape.internal(world, shapeIndex1, shapeWorldAndGeneration);
  }

  /// Creates a chain of one-sided segments and attaches it to the body.
  ///
  /// Chains need at least four points: open chains use the first and last
  /// point as invisible ghost anchors that smooth collisions at the ends.
  ///
  /// Cannot be called while the world is stepping.
  Chain createChain(ChainDef definition) {
    world.checkCanMutate('create a chain');
    if (definition.points.length < 4) {
      throw ArgumentError('A chain needs at least four points');
    }
    final materialCount = definition.materials.length;
    if (materialCount != 1 && materialCount != definition.points.length) {
      throw ArgumentError(
        'A chain needs one material, or one material per point',
      );
    }
    final (chainIndex1, chainWorldAndGeneration) = rawBox2D.createChain(
      index1,
      worldAndGeneration,
      points: [
        for (final point in definition.points) ...[point.x, point.y],
      ],
      materials: [
        for (final material in definition.materials) ...[
          material.friction,
          material.restitution,
          material.rollingResistance,
          material.tangentSpeed,
          material.userMaterialId.toDouble(),
          material.customColor.toDouble(),
        ],
      ],
      categoryBits: definition.filter.categoryBits,
      maskBits: definition.filter.maskBits,
      groupIndex: definition.filter.groupIndex,
      isLoop: definition.isLoop,
      enableSensorEvents: definition.enableSensorEvents,
    );
    if (definition.userData != null) {
      world.chainUserData[(chainIndex1, chainWorldAndGeneration)] =
          definition.userData;
    }
    world.chainOwners[(chainIndex1, chainWorldAndGeneration)] = (
      index1,
      worldAndGeneration,
    );
    return Chain.internal(world, chainIndex1, chainWorldAndGeneration);
  }

  /// The shapes attached to this body.
  List<Shape> get shapes {
    final ids = rawBox2D.bodyGetShapes(index1, worldAndGeneration);
    return [
      for (var i = 0; i < ids.length; i += 2)
        Shape.internal(world, ids[i], ids[i + 1]),
    ];
  }

  /// The joints attached to this body.
  List<Joint> get joints {
    final ids = rawBox2D.bodyGetJoints(index1, worldAndGeneration);
    return [
      for (var i = 0; i < ids.length; i += 2)
        jointFromId(world, ids[i], ids[i + 1]),
    ];
  }

  /// Destroys this body and all its shapes, chains, and joints.
  ///
  /// Safe to call while the world is stepping (from a callback or while
  /// processing events): the destruction is deferred until the step ends,
  /// and the body stays valid until then.
  void destroy() {
    if (world.locked) {
      world.deferredActions.add(destroy);
      return;
    }
    if (!isValid) {
      // Also covers a deferred destroy that was requested more than once.
      return;
    }
    for (final shape in shapes) {
      world.shapeUserData.remove((shape.index1, shape.worldAndGeneration));
    }
    for (final joint in joints) {
      world.jointUserData.remove((joint.index1, joint.worldAndGeneration));
    }
    world.chainOwners.removeWhere((chainId, ownerId) {
      if (ownerId == (index1, worldAndGeneration)) {
        world.chainUserData.remove(chainId);
        return true;
      }
      return false;
    });
    world.bodyUserData.remove((index1, worldAndGeneration));
    rawBox2D.destroyBody(index1, worldAndGeneration);
  }

  RawShapeDef _rawShapeDef(ShapeDef definition) => (
    friction: definition.material.friction,
    restitution: definition.material.restitution,
    rollingResistance: definition.material.rollingResistance,
    tangentSpeed: definition.material.tangentSpeed,
    userMaterialId: definition.material.userMaterialId,
    customColor: definition.material.customColor,
    density: definition.density,
    categoryBits: definition.filter.categoryBits,
    maskBits: definition.filter.maskBits,
    groupIndex: definition.filter.groupIndex,
    isSensor: definition.isSensor,
    enableSensorEvents: definition.enableSensorEvents,
    enableContactEvents: definition.enableContactEvents,
    enableHitEvents: definition.enableHitEvents,
    enablePreSolveEvents: definition.enablePreSolveEvents,
    invokeContactCreation: definition.invokeContactCreation,
    updateBodyMass: definition.updateBodyMass,
  );

  @override
  bool operator ==(Object other) =>
      other is Body &&
      other.world == world &&
      other.index1 == index1 &&
      other.worldAndGeneration == worldAndGeneration;

  @override
  int get hashCode => Object.hash(world, index1, worldAndGeneration);

  @override
  String toString() => 'Body($index1)';
}
