import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/math.dart';
import 'package:forge2d/src/api/world.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';
import 'package:vector_math/vector_math.dart';

/// A collision shape attached to a body.
///
/// Wraps a native `b2ShapeId`. A [Shape] instance is a cheap value-like
/// handle: two handles to the same shape compare equal, and holding one does
/// not keep the shape alive.
@immutable
class Shape {
  /// Wraps an existing native shape id. Internal to forge2d.
  @internal
  const Shape.internal(this.world, this.index1, this.wg);

  /// The world this shape belongs to.
  final World world;

  /// The first half of the packed native shape id.
  @internal
  final int index1;

  /// The second half of the packed native shape id.
  @internal
  final int wg;

  /// Whether this shape has not been destroyed.
  bool get isValid => rawBox2D.shapeIsValid(index1, wg);

  /// The kind of geometry this shape has.
  ShapeType get type => ShapeType.values[rawBox2D.shapeGetType(index1, wg)];

  /// The body this shape is attached to.
  Body get body {
    final (bodyIndex1, bodyWg) = rawBox2D.shapeGetBody(index1, wg);
    return Body.internal(world, bodyIndex1, bodyWg);
  }

  /// Whether the shape is a sensor.
  ///
  /// Sensors detect overlap but produce no collision response. This cannot
  /// be changed after creation.
  bool get isSensor => rawBox2D.shapeIsSensor(index1, wg);

  /// The density of the shape, in kilograms per square meter.
  ///
  /// Setting the density updates the body mass; use [setDensity] to defer
  /// that.
  double get density => rawBox2D.shapeGetDensity(index1, wg);

  set density(double value) =>
      rawBox2D.shapeSetDensity(index1, wg, value, updateBodyMass: true);

  /// Sets the density, optionally skipping the body mass update. Call
  /// [Body.applyMassFromShapes] afterwards when skipping.
  void setDensity(double value, {required bool updateBodyMass}) => rawBox2D
      .shapeSetDensity(index1, wg, value, updateBodyMass: updateBodyMass);

  /// The friction coefficient of the shape.
  double get friction => rawBox2D.shapeGetFriction(index1, wg);

  set friction(double value) => rawBox2D.shapeSetFriction(index1, wg, value);

  /// The restitution (bounciness) of the shape.
  double get restitution => rawBox2D.shapeGetRestitution(index1, wg);

  set restitution(double value) =>
      rawBox2D.shapeSetRestitution(index1, wg, value);

  /// The collision filter of the shape.
  Filter get filter {
    final (categoryBits, maskBits, groupIndex) = rawBox2D.shapeGetFilter(
      index1,
      wg,
    );
    return Filter(
      categoryBits: categoryBits,
      maskBits: maskBits,
      groupIndex: groupIndex,
    );
  }

  set filter(Filter value) => rawBox2D.shapeSetFilter(
    index1,
    wg,
    value.categoryBits,
    value.maskBits,
    value.groupIndex,
  );

  /// Whether the shape reports sensor overlap events.
  bool get sensorEventsEnabled =>
      rawBox2D.shapeAreSensorEventsEnabled(index1, wg);

  set sensorEventsEnabled(bool value) =>
      rawBox2D.shapeEnableSensorEvents(index1, wg, enabled: value);

  /// Whether the shape reports contact begin and end events.
  bool get contactEventsEnabled =>
      rawBox2D.shapeAreContactEventsEnabled(index1, wg);

  set contactEventsEnabled(bool value) =>
      rawBox2D.shapeEnableContactEvents(index1, wg, enabled: value);

  /// Whether the shape reports contact hit events.
  bool get hitEventsEnabled => rawBox2D.shapeAreHitEventsEnabled(index1, wg);

  set hitEventsEnabled(bool value) =>
      rawBox2D.shapeEnableHitEvents(index1, wg, enabled: value);

  /// Whether the shape reports pre-solve events.
  bool get preSolveEventsEnabled =>
      rawBox2D.shapeArePreSolveEventsEnabled(index1, wg);

  set preSolveEventsEnabled(bool value) =>
      rawBox2D.shapeEnablePreSolveEvents(index1, wg, enabled: value);

  /// Whether the given world point is inside the shape.
  bool testPoint(Vector2 point) =>
      rawBox2D.shapeTestPoint(index1, wg, point.x, point.y);

  /// The current world axis-aligned bounding box of the shape.
  Aabb get aabb {
    final (lowerX, lowerY, upperX, upperY) = rawBox2D.shapeGetAabb(
      index1,
      wg,
    );
    return Aabb(Vector2(lowerX, lowerY), Vector2(upperX, upperY));
  }

  /// The user data associated with the shape.
  Object? get userData => world.shapeUserData[(index1, wg)];

  set userData(Object? value) {
    if (value == null) {
      world.shapeUserData.remove((index1, wg));
    } else {
      world.shapeUserData[(index1, wg)] = value;
    }
  }

  /// Destroys this shape and removes it from its body.
  ///
  /// When [updateBodyMass] is false, call [Body.applyMassFromShapes]
  /// afterwards.
  void destroy({bool updateBodyMass = true}) {
    world.shapeUserData.remove((index1, wg));
    rawBox2D.destroyShape(index1, wg, updateBodyMass: updateBodyMass);
  }

  @override
  bool operator ==(Object other) =>
      other is Shape &&
      other.world == world &&
      other.index1 == index1 &&
      other.wg == wg;

  @override
  int get hashCode => Object.hash(world, index1, wg);

  @override
  String toString() => 'Shape($index1)';
}
