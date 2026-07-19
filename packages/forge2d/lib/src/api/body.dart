import 'package:forge2d/src/api/chain.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/geometry.dart';
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
  const Body.internal(this.world, this.index1, this.wg);

  /// The world this body belongs to.
  final World world;

  /// The first half of the packed native body id.
  @internal
  final int index1;

  /// The second half of the packed native body id.
  @internal
  final int wg;

  /// Whether this body has not been destroyed.
  bool get isValid => rawBox2D.bodyIsValid(index1, wg);

  /// The world position of the body origin.
  Vector2 get position {
    final (x, y) = rawBox2D.bodyGetPosition(index1, wg);
    return Vector2(x, y);
  }

  /// The world rotation of the body.
  Rot get rotation {
    final (cos, sin) = rawBox2D.bodyGetRotation(index1, wg);
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
      wg,
      position.x,
      position.y,
      rotation.cos,
      rotation.sin,
    );
  }

  /// The linear velocity of the body origin, in meters per second.
  Vector2 get linearVelocity {
    final (x, y) = rawBox2D.bodyGetLinearVelocity(index1, wg);
    return Vector2(x, y);
  }

  set linearVelocity(Vector2 value) =>
      rawBox2D.bodySetLinearVelocity(index1, wg, value.x, value.y);

  /// The angular velocity of the body, in radians per second.
  double get angularVelocity => rawBox2D.bodyGetAngularVelocity(index1, wg);

  set angularVelocity(double value) =>
      rawBox2D.bodySetAngularVelocity(index1, wg, value);

  /// Applies [force] at world [point], or at the center of mass when
  /// [point] is omitted.
  ///
  /// Forces are applied continuously over the time step. When [wake] is
  /// false and the body is sleeping, the force is ignored.
  void applyForce(Vector2 force, {Vector2? point, bool wake = true}) {
    if (point == null) {
      rawBox2D.bodyApplyForceToCenter(index1, wg, force.x, force.y, wake: wake);
    } else {
      rawBox2D.bodyApplyForce(
        index1,
        wg,
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
      rawBox2D.bodyApplyTorque(index1, wg, torque, wake: wake);

  /// Applies [impulse] at world [point], or at the center of mass when
  /// [point] is omitted, immediately changing the linear velocity.
  void applyLinearImpulse(Vector2 impulse, {Vector2? point, bool wake = true}) {
    if (point == null) {
      rawBox2D.bodyApplyLinearImpulseToCenter(
        index1,
        wg,
        impulse.x,
        impulse.y,
        wake: wake,
      );
    } else {
      rawBox2D.bodyApplyLinearImpulse(
        index1,
        wg,
        impulse.x,
        impulse.y,
        point.x,
        point.y,
        wake: wake,
      );
    }
  }

  /// Applies an angular impulse, immediately changing the angular velocity.
  void applyAngularImpulse(double impulse, {bool wake = true}) =>
      rawBox2D.bodyApplyAngularImpulse(index1, wg, impulse, wake: wake);

  /// The mass of the body, in kilograms.
  double get mass => rawBox2D.bodyGetMass(index1, wg);

  /// The rotational inertia about the center of mass, in kg*m^2.
  double get rotationalInertia => rawBox2D.bodyGetRotationalInertia(index1, wg);

  /// The center of mass relative to the body origin.
  Vector2 get localCenterOfMass {
    final (x, y) = rawBox2D.bodyGetLocalCenterOfMass(index1, wg);
    return Vector2(x, y);
  }

  /// The center of mass in world coordinates.
  Vector2 get worldCenterOfMass {
    final (x, y) = rawBox2D.bodyGetWorldCenterOfMass(index1, wg);
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
    wg,
    value.mass,
    value.rotationalInertia,
    value.center.x,
    value.center.y,
  );

  /// Recomputes the mass properties from the attached shapes, discarding any
  /// override set through [massData].
  void applyMassFromShapes() => rawBox2D.bodyApplyMassFromShapes(index1, wg);

  /// The body type: static, kinematic, or dynamic.
  ///
  /// Changing the type is expensive: it wakes touching bodies and recreates
  /// contact data.
  BodyType get type => BodyType.values[rawBox2D.bodyGetType(index1, wg)];

  set type(BodyType value) => rawBox2D.bodySetType(index1, wg, value.index);

  /// The body name, for debugging. Empty when unset.
  String get name => rawBox2D.bodyGetName(index1, wg);

  set name(String? value) => rawBox2D.bodySetName(index1, wg, value);

  /// The user data associated with the body.
  Object? get userData => world.bodyUserData[(index1, wg)];

  set userData(Object? value) {
    if (value == null) {
      world.bodyUserData.remove((index1, wg));
    } else {
      world.bodyUserData[(index1, wg)] = value;
    }
  }

  /// Whether the body is awake and being simulated.
  bool get isAwake => rawBox2D.bodyIsAwake(index1, wg);

  set isAwake(bool value) => rawBox2D.bodySetAwake(index1, wg, awake: value);

  /// Whether the body may fall asleep when it comes to rest.
  bool get sleepEnabled => rawBox2D.bodyIsSleepEnabled(index1, wg);

  set sleepEnabled(bool value) =>
      rawBox2D.bodyEnableSleep(index1, wg, enabled: value);

  /// The speed below which the body may fall asleep, in meters per second.
  double get sleepThreshold => rawBox2D.bodyGetSleepThreshold(index1, wg);

  set sleepThreshold(double value) =>
      rawBox2D.bodySetSleepThreshold(index1, wg, value);

  /// Whether the body participates in the simulation.
  ///
  /// Disabling a body is expensive: it removes the body from the broad
  /// phase and destroys its contacts.
  bool get isEnabled => rawBox2D.bodyIsEnabled(index1, wg);

  set isEnabled(bool value) {
    if (value) {
      rawBox2D.bodyEnable(index1, wg);
    } else {
      rawBox2D.bodyDisable(index1, wg);
    }
  }

  /// Whether the body rotation is fixed.
  bool get fixedRotation => rawBox2D.bodyIsFixedRotation(index1, wg);

  set fixedRotation(bool value) =>
      rawBox2D.bodySetFixedRotation(index1, wg, flag: value);

  /// Whether the body uses continuous collision detection against static
  /// and kinematic bodies.
  bool get isBullet => rawBox2D.bodyIsBullet(index1, wg);

  set isBullet(bool value) => rawBox2D.bodySetBullet(index1, wg, flag: value);

  /// The scale applied to world gravity for this body.
  double get gravityScale => rawBox2D.bodyGetGravityScale(index1, wg);

  set gravityScale(double value) =>
      rawBox2D.bodySetGravityScale(index1, wg, value);

  /// Damping applied to the linear velocity.
  double get linearDamping => rawBox2D.bodyGetLinearDamping(index1, wg);

  set linearDamping(double value) =>
      rawBox2D.bodySetLinearDamping(index1, wg, value);

  /// Damping applied to the angular velocity.
  double get angularDamping => rawBox2D.bodyGetAngularDamping(index1, wg);

  set angularDamping(double value) =>
      rawBox2D.bodySetAngularDamping(index1, wg, value);

  /// Converts a point on the body to world coordinates.
  Vector2 worldPoint(Vector2 localPoint) {
    final (x, y) = rawBox2D.bodyGetWorldPoint(
      index1,
      wg,
      localPoint.x,
      localPoint.y,
    );
    return Vector2(x, y);
  }

  /// Converts a world point to the body's local coordinates.
  Vector2 localPoint(Vector2 worldPoint) {
    final (x, y) = rawBox2D.bodyGetLocalPoint(
      index1,
      wg,
      worldPoint.x,
      worldPoint.y,
    );
    return Vector2(x, y);
  }

  /// Creates a shape with the given [geometry] and attaches it to the body.
  Shape createShape(ShapeGeometry geometry, [ShapeDef? def]) {
    final shapeDef = def ?? ShapeDef();
    final rawDef = _rawShapeDef(shapeDef);
    final (shapeIndex1, shapeWg) = switch (geometry) {
      Circle(:final center, :final radius) => rawBox2D.createCircleShape(
        index1,
        wg,
        centerX: center.x,
        centerY: center.y,
        radius: radius,
        def: rawDef,
      ),
      Capsule(:final center1, :final center2, :final radius) =>
        rawBox2D.createCapsuleShape(
          index1,
          wg,
          center1X: center1.x,
          center1Y: center1.y,
          center2X: center2.x,
          center2Y: center2.y,
          radius: radius,
          def: rawDef,
        ),
      Segment(:final point1, :final point2) => rawBox2D.createSegmentShape(
        index1,
        wg,
        point1X: point1.x,
        point1Y: point1.y,
        point2X: point2.x,
        point2Y: point2.y,
        def: rawDef,
      ),
      Polygon(points: final points?, :final radius) =>
        rawBox2D.createPolygonShape(
          index1,
          wg,
          points: [
            for (final point in points) ...[point.x, point.y],
          ],
          radius: radius,
          def: rawDef,
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
          wg,
          halfWidth: halfWidth,
          halfHeight: halfHeight,
          centerX: center?.x ?? 0,
          centerY: center?.y ?? 0,
          rotationCos: rotation == null ? 1 : Rot.fromAngle(rotation).cos,
          rotationSin: rotation == null ? 0 : Rot.fromAngle(rotation).sin,
          radius: radius,
          def: rawDef,
        ),
      Polygon() => throw StateError('Unreachable polygon configuration'),
    };
    if (shapeDef.userData != null) {
      world.shapeUserData[(shapeIndex1, shapeWg)] = shapeDef.userData;
    }
    return Shape.internal(world, shapeIndex1, shapeWg);
  }

  /// Creates a chain of one-sided segments and attaches it to the body.
  Chain createChain(ChainDef def) {
    final (chainIndex1, chainWg) = rawBox2D.createChain(
      index1,
      wg,
      points: [
        for (final point in def.points) ...[point.x, point.y],
      ],
      materials: [
        for (final material in def.materials) ...[
          material.friction,
          material.restitution,
          material.rollingResistance,
          material.tangentSpeed,
          material.userMaterialId.toDouble(),
          material.customColor.toDouble(),
        ],
      ],
      categoryBits: def.filter.categoryBits,
      maskBits: def.filter.maskBits,
      groupIndex: def.filter.groupIndex,
      isLoop: def.isLoop,
      enableSensorEvents: def.enableSensorEvents,
    );
    if (def.userData != null) {
      world.chainUserData[(chainIndex1, chainWg)] = def.userData;
    }
    return Chain.internal(world, chainIndex1, chainWg);
  }

  /// The shapes attached to this body.
  List<Shape> get shapes {
    final ids = rawBox2D.bodyGetShapes(index1, wg);
    return [
      for (var i = 0; i < ids.length; i += 2)
        Shape.internal(world, ids[i], ids[i + 1]),
    ];
  }

  /// Destroys this body and all its shapes and joints.
  void destroy() {
    for (final shape in shapes) {
      world.shapeUserData.remove((shape.index1, shape.wg));
    }
    world.bodyUserData.remove((index1, wg));
    rawBox2D.destroyBody(index1, wg);
  }

  RawShapeDef _rawShapeDef(ShapeDef def) => (
    friction: def.material.friction,
    restitution: def.material.restitution,
    rollingResistance: def.material.rollingResistance,
    tangentSpeed: def.material.tangentSpeed,
    userMaterialId: def.material.userMaterialId,
    customColor: def.material.customColor,
    density: def.density,
    categoryBits: def.filter.categoryBits,
    maskBits: def.filter.maskBits,
    groupIndex: def.filter.groupIndex,
    isSensor: def.isSensor,
    enableSensorEvents: def.enableSensorEvents,
    enableContactEvents: def.enableContactEvents,
    enableHitEvents: def.enableHitEvents,
    enablePreSolveEvents: def.enablePreSolveEvents,
    invokeContactCreation: def.invokeContactCreation,
    updateBodyMass: def.updateBodyMass,
  );

  @override
  bool operator ==(Object other) =>
      other is Body &&
      other.world == world &&
      other.index1 == index1 &&
      other.wg == wg;

  @override
  int get hashCode => Object.hash(world, index1, wg);

  @override
  String toString() => 'Body($index1)';
}
