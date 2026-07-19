import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/joints/distance_joint.dart';
import 'package:forge2d/src/api/joints/filter_joint.dart';
import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/api/joints/motor_joint.dart';
import 'package:forge2d/src/api/joints/mouse_joint.dart';
import 'package:forge2d/src/api/joints/prismatic_joint.dart';
import 'package:forge2d/src/api/joints/revolute_joint.dart';
import 'package:forge2d/src/api/joints/weld_joint.dart';
import 'package:forge2d/src/api/joints/wheel_joint.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// The simulation world: a container for bodies, shapes, and joints.
///
/// Wraps a native `b2WorldId`. Worlds are created up front and destroyed
/// explicitly with [destroy]; destroying a world frees every body, shape,
/// and joint in it.
///
/// A world must only be used from the isolate that created it.
class World {
  /// Creates a world.
  ///
  /// [gravity] is a convenience override for the most commonly changed
  /// setting; use [def] for full control over the remaining `b2WorldDef`
  /// fields.
  World({Vector2? gravity, WorldDef? def})
    : this._(gravity: gravity, def: def ?? WorldDef());

  World._({required WorldDef def, Vector2? gravity})
    : id = rawBox2D.createWorld(
        gravityX: (gravity ?? def.gravity).x,
        gravityY: (gravity ?? def.gravity).y,
        restitutionThreshold: def.restitutionThreshold,
        hitEventThreshold: def.hitEventThreshold,
        contactHertz: def.contactHertz,
        contactDampingRatio: def.contactDampingRatio,
        maxContactPushSpeed: def.maxContactPushSpeed,
        maximumLinearSpeed: def.maximumLinearSpeed,
        enableSleep: def.enableSleep,
        enableContinuous: def.enableContinuous,
      );

  /// The packed native world id.
  @internal
  final int id;

  /// Dart-side user data for the bodies of this world, keyed by body id.
  @internal
  final Map<(int, int), Object?> bodyUserData = {};

  /// Dart-side user data for the shapes of this world, keyed by shape id.
  @internal
  final Map<(int, int), Object?> shapeUserData = {};

  /// Dart-side user data for the chains of this world, keyed by chain id.
  @internal
  final Map<(int, int), Object?> chainUserData = {};

  /// Dart-side user data for the joints of this world, keyed by joint id.
  @internal
  final Map<(int, int), Object?> jointUserData = {};

  /// Whether this world has not been destroyed.
  bool get isValid => rawBox2D.worldIsValid(id);

  /// Advances the simulation by [timeStep] seconds.
  ///
  /// Use a fixed time step, usually 1/60. [subStepCount] trades performance
  /// for accuracy; 4 is a good default.
  void step(double timeStep, {int subStepCount = 4}) {
    assert(isValid, 'World has been destroyed');
    rawBox2D.worldStep(id, timeStep, subStepCount);
  }

  /// The gravity vector, in meters per second squared.
  Vector2 get gravity {
    final (x, y) = rawBox2D.worldGetGravity(id);
    return Vector2(x, y);
  }

  set gravity(Vector2 value) => rawBox2D.worldSetGravity(id, value.x, value.y);

  /// Whether bodies in this world may fall asleep.
  ///
  /// Disabling sleep wakes every sleeping body.
  bool get sleepingEnabled => rawBox2D.worldIsSleepingEnabled(id);

  set sleepingEnabled(bool value) =>
      rawBox2D.worldEnableSleeping(id, enabled: value);

  /// Whether continuous collision detection runs for fast bodies.
  bool get continuousEnabled => rawBox2D.worldIsContinuousEnabled(id);

  set continuousEnabled(bool value) =>
      rawBox2D.worldEnableContinuous(id, enabled: value);

  /// Creates a body in this world.
  Body createBody([BodyDef? def]) {
    assert(isValid, 'World has been destroyed');
    final bodyDef = def ?? BodyDef();
    final (index1, wg) = rawBox2D.createBody(
      id,
      type: bodyDef.type.index,
      positionX: bodyDef.position.x,
      positionY: bodyDef.position.y,
      rotationCos: bodyDef.rotation.cos,
      rotationSin: bodyDef.rotation.sin,
      linearVelocityX: bodyDef.linearVelocity.x,
      linearVelocityY: bodyDef.linearVelocity.y,
      angularVelocity: bodyDef.angularVelocity,
      linearDamping: bodyDef.linearDamping,
      angularDamping: bodyDef.angularDamping,
      gravityScale: bodyDef.gravityScale,
      sleepThreshold: bodyDef.sleepThreshold,
      name: bodyDef.name,
      enableSleep: bodyDef.enableSleep,
      isAwake: bodyDef.isAwake,
      fixedRotation: bodyDef.fixedRotation,
      isBullet: bodyDef.isBullet,
      isEnabled: bodyDef.isEnabled,
      allowFastRotation: bodyDef.allowFastRotation,
    );
    if (bodyDef.userData != null) {
      bodyUserData[(index1, wg)] = bodyDef.userData;
    }
    return Body.internal(this, index1, wg);
  }

  /// Creates a distance joint from [def].
  DistanceJoint createDistanceJoint(DistanceJointDef def) {
    final (index1, wg) = rawBox2D.createDistanceJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      localAnchorA: (def.localAnchorA.x, def.localAnchorA.y),
      localAnchorB: (def.localAnchorB.x, def.localAnchorB.y),
      length: def.length,
      enableSpring: def.enableSpring,
      hertz: def.hertz,
      dampingRatio: def.dampingRatio,
      enableLimit: def.enableLimit,
      minLength: def.minLength,
      maxLength: def.maxLength,
      enableMotor: def.enableMotor,
      maxMotorForce: def.maxMotorForce,
      motorSpeed: def.motorSpeed,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return DistanceJoint.internal(this, index1, wg);
  }

  /// Creates a filter joint from [def], disabling collision between the two
  /// bodies.
  FilterJoint createFilterJoint(FilterJointDef def) {
    final (index1, wg) = rawBox2D.createFilterJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
    );
    _storeJointUserData(index1, wg, def);
    return FilterJoint.internal(this, index1, wg);
  }

  /// Creates a motor joint from [def].
  MotorJoint createMotorJoint(MotorJointDef def) {
    final (index1, wg) = rawBox2D.createMotorJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      linearOffset: (def.linearOffset.x, def.linearOffset.y),
      angularOffset: def.angularOffset,
      maxForce: def.maxForce,
      maxTorque: def.maxTorque,
      correctionFactor: def.correctionFactor,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return MotorJoint.internal(this, index1, wg);
  }

  /// Creates a mouse joint from [def].
  MouseJoint createMouseJoint(MouseJointDef def) {
    final (index1, wg) = rawBox2D.createMouseJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      target: (def.target.x, def.target.y),
      hertz: def.hertz,
      dampingRatio: def.dampingRatio,
      maxForce: def.maxForce,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return MouseJoint.internal(this, index1, wg);
  }

  /// Creates a prismatic joint from [def].
  PrismaticJoint createPrismaticJoint(PrismaticJointDef def) {
    final (index1, wg) = rawBox2D.createPrismaticJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      localAnchorA: (def.localAnchorA.x, def.localAnchorA.y),
      localAnchorB: (def.localAnchorB.x, def.localAnchorB.y),
      localAxisA: (def.localAxisA.x, def.localAxisA.y),
      referenceAngle: def.referenceAngle,
      targetTranslation: def.targetTranslation,
      enableSpring: def.enableSpring,
      hertz: def.hertz,
      dampingRatio: def.dampingRatio,
      enableLimit: def.enableLimit,
      lowerTranslation: def.lowerTranslation,
      upperTranslation: def.upperTranslation,
      enableMotor: def.enableMotor,
      maxMotorForce: def.maxMotorForce,
      motorSpeed: def.motorSpeed,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return PrismaticJoint.internal(this, index1, wg);
  }

  /// Creates a revolute joint from [def].
  RevoluteJoint createRevoluteJoint(RevoluteJointDef def) {
    final (index1, wg) = rawBox2D.createRevoluteJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      localAnchorA: (def.localAnchorA.x, def.localAnchorA.y),
      localAnchorB: (def.localAnchorB.x, def.localAnchorB.y),
      referenceAngle: def.referenceAngle,
      targetAngle: def.targetAngle,
      enableSpring: def.enableSpring,
      hertz: def.hertz,
      dampingRatio: def.dampingRatio,
      enableLimit: def.enableLimit,
      lowerAngle: def.lowerAngle,
      upperAngle: def.upperAngle,
      enableMotor: def.enableMotor,
      maxMotorTorque: def.maxMotorTorque,
      motorSpeed: def.motorSpeed,
      drawSize: def.drawSize,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return RevoluteJoint.internal(this, index1, wg);
  }

  /// Creates a weld joint from [def].
  WeldJoint createWeldJoint(WeldJointDef def) {
    final (index1, wg) = rawBox2D.createWeldJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      localAnchorA: (def.localAnchorA.x, def.localAnchorA.y),
      localAnchorB: (def.localAnchorB.x, def.localAnchorB.y),
      referenceAngle: def.referenceAngle,
      linearHertz: def.linearHertz,
      angularHertz: def.angularHertz,
      linearDampingRatio: def.linearDampingRatio,
      angularDampingRatio: def.angularDampingRatio,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return WeldJoint.internal(this, index1, wg);
  }

  /// Creates a wheel joint from [def].
  WheelJoint createWheelJoint(WheelJointDef def) {
    final (index1, wg) = rawBox2D.createWheelJoint(
      id,
      bodyA: (def.bodyA.index1, def.bodyA.wg),
      bodyB: (def.bodyB.index1, def.bodyB.wg),
      localAnchorA: (def.localAnchorA.x, def.localAnchorA.y),
      localAnchorB: (def.localAnchorB.x, def.localAnchorB.y),
      localAxisA: (def.localAxisA.x, def.localAxisA.y),
      enableSpring: def.enableSpring,
      hertz: def.hertz,
      dampingRatio: def.dampingRatio,
      enableLimit: def.enableLimit,
      lowerTranslation: def.lowerTranslation,
      upperTranslation: def.upperTranslation,
      enableMotor: def.enableMotor,
      maxMotorTorque: def.maxMotorTorque,
      motorSpeed: def.motorSpeed,
      collideConnected: def.collideConnected,
    );
    _storeJointUserData(index1, wg, def);
    return WheelJoint.internal(this, index1, wg);
  }

  void _storeJointUserData(int index1, int wg, JointDef def) {
    if (def.userData != null) {
      jointUserData[(index1, wg)] = def.userData;
    }
  }

  /// Destroys this world and everything in it.
  void destroy() {
    assert(isValid, 'World has been destroyed');
    rawBox2D.destroyWorld(id);
    bodyUserData.clear();
    shapeUserData.clear();
    chainUserData.clear();
    jointUserData.clear();
  }
}
