import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/joints/distance_joint.dart';
import 'package:forge2d/src/api/joints/filter_joint.dart';
import 'package:forge2d/src/api/joints/motor_joint.dart';
import 'package:forge2d/src/api/joints/mouse_joint.dart';
import 'package:forge2d/src/api/joints/prismatic_joint.dart';
import 'package:forge2d/src/api/joints/revolute_joint.dart';
import 'package:forge2d/src/api/joints/weld_joint.dart';
import 'package:forge2d/src/api/joints/wheel_joint.dart';
import 'package:forge2d/src/api/world.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// Wraps an existing joint id in the wrapper class matching its native
/// type. Internal to forge2d.
@internal
Joint jointFromId(World world, int index1, int wg) {
  final type = JointType.values[rawBox2D.jointGetType(index1, wg)];
  return switch (type) {
    JointType.distance => DistanceJoint.internal(world, index1, wg),
    JointType.filter => FilterJoint.internal(world, index1, wg),
    JointType.motor => MotorJoint.internal(world, index1, wg),
    JointType.mouse => MouseJoint.internal(world, index1, wg),
    JointType.prismatic => PrismaticJoint.internal(world, index1, wg),
    JointType.revolute => RevoluteJoint.internal(world, index1, wg),
    JointType.weld => WeldJoint.internal(world, index1, wg),
    JointType.wheel => WheelJoint.internal(world, index1, wg),
  };
}

/// The data shared by all joint definitions: the two bodies to connect and
/// whether they may keep colliding with each other.
abstract class JointDef {
  /// Creates a joint definition connecting [bodyA] and [bodyB].
  JointDef({
    required this.bodyA,
    required this.bodyB,
    this.collideConnected = false,
    this.userData,
  });

  /// The first attached body.
  Body bodyA;

  /// The second attached body.
  Body bodyB;

  /// Whether the connected bodies may collide with each other.
  bool collideConnected;

  /// An arbitrary object associated with the joint.
  ///
  /// Stored on the Dart side; the native user data pointer is not used.
  Object? userData;
}

/// A constraint connecting two bodies.
///
/// Wraps a native `b2JointId`. A [Joint] instance is a cheap value-like
/// handle: two handles to the same joint compare equal, and holding one does
/// not keep the joint alive. Destroy joints explicitly with [destroy];
/// destroying either attached body also destroys the joint.
@immutable
abstract base class Joint {
  /// Wraps an existing native joint id. Internal to forge2d.
  @internal
  const Joint.internal(this.world, this.index1, this.wg);

  /// The world this joint belongs to.
  final World world;

  /// The first half of the packed native joint id.
  @internal
  final int index1;

  /// The second half of the packed native joint id.
  @internal
  final int wg;

  /// Whether this joint has not been destroyed.
  bool get isValid => rawBox2D.jointIsValid(index1, wg);

  /// The type of this joint.
  JointType get type => JointType.values[rawBox2D.jointGetType(index1, wg)];

  /// The first attached body.
  Body get bodyA {
    final (bodyIndex1, bodyWg) = rawBox2D.jointGetBodyA(index1, wg);
    return Body.internal(world, bodyIndex1, bodyWg);
  }

  /// The second attached body.
  Body get bodyB {
    final (bodyIndex1, bodyWg) = rawBox2D.jointGetBodyB(index1, wg);
    return Body.internal(world, bodyIndex1, bodyWg);
  }

  /// The joint anchor point on the first body, in its local coordinates.
  Vector2 get localAnchorA {
    final (x, y) = rawBox2D.jointGetLocalAnchorA(index1, wg);
    return Vector2(x, y);
  }

  /// The joint anchor point on the second body, in its local coordinates.
  Vector2 get localAnchorB {
    final (x, y) = rawBox2D.jointGetLocalAnchorB(index1, wg);
    return Vector2(x, y);
  }

  /// Whether the connected bodies may collide with each other.
  bool get collideConnected => rawBox2D.jointGetCollideConnected(index1, wg);

  set collideConnected(bool value) =>
      rawBox2D.jointSetCollideConnected(index1, wg, value: value);

  /// Wakes both connected bodies.
  void wakeBodies() => rawBox2D.jointWakeBodies(index1, wg);

  /// The constraint force applied by the joint, in newtons.
  Vector2 get constraintForce {
    final (x, y) = rawBox2D.jointGetConstraintForce(index1, wg);
    return Vector2(x, y);
  }

  /// The constraint torque applied by the joint, in newton-meters.
  double get constraintTorque => rawBox2D.jointGetConstraintTorque(index1, wg);

  /// The user data associated with the joint.
  Object? get userData => world.jointUserData[(index1, wg)];

  set userData(Object? value) {
    if (value == null) {
      world.jointUserData.remove((index1, wg));
    } else {
      world.jointUserData[(index1, wg)] = value;
    }
  }

  /// Destroys this joint.
  void destroy() {
    world.jointUserData.remove((index1, wg));
    rawBox2D.destroyJoint(index1, wg);
  }

  @override
  bool operator ==(Object other) =>
      other is Joint &&
      other.world == world &&
      other.index1 == index1 &&
      other.wg == wg;

  @override
  int get hashCode => Object.hash(world, index1, wg);

  @override
  String toString() => '${type.name} Joint($index1)';
}
