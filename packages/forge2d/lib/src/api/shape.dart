import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/geometry.dart';
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
  const Shape.internal(this.world, this.index1, this.worldAndGeneration);

  /// The world this shape belongs to.
  final World world;

  /// The first half of the packed native shape id.
  @internal
  final int index1;

  /// The second half of the packed native shape id.
  @internal
  final int worldAndGeneration;

  /// Whether this shape has not been destroyed.
  bool get isValid => rawBox2D.shapeIsValid(index1, worldAndGeneration);

  /// The kind of geometry this shape has.
  ShapeType get type =>
      ShapeType.values[rawBox2D.shapeGetType(index1, worldAndGeneration)];

  /// The body this shape is attached to.
  Body get body {
    final (bodyIndex1, bodyWorldAndGeneration) = rawBox2D.shapeGetBody(
      index1,
      worldAndGeneration,
    );
    return Body.internal(world, bodyIndex1, bodyWorldAndGeneration);
  }

  /// Whether the shape is a sensor.
  ///
  /// Sensors detect overlap but produce no collision response. This cannot
  /// be changed after creation.
  bool get isSensor => rawBox2D.shapeIsSensor(index1, worldAndGeneration);

  /// The density of the shape, in kilograms per square meter.
  ///
  /// Setting the density updates the body mass; use [setDensity] to defer
  /// that.
  double get density => rawBox2D.shapeGetDensity(index1, worldAndGeneration);

  set density(double value) => rawBox2D.shapeSetDensity(
    index1,
    worldAndGeneration,
    value,
    updateBodyMass: true,
  );

  /// Sets the density, optionally skipping the body mass update. Call
  /// [Body.applyMassFromShapes] afterwards when skipping.
  void setDensity(double value, {required bool updateBodyMass}) =>
      rawBox2D.shapeSetDensity(
        index1,
        worldAndGeneration,
        value,
        updateBodyMass: updateBodyMass,
      );

  /// The friction coefficient of the shape.
  double get friction => rawBox2D.shapeGetFriction(index1, worldAndGeneration);

  set friction(double value) =>
      rawBox2D.shapeSetFriction(index1, worldAndGeneration, value);

  /// The restitution (bounciness) of the shape.
  double get restitution =>
      rawBox2D.shapeGetRestitution(index1, worldAndGeneration);

  set restitution(double value) =>
      rawBox2D.shapeSetRestitution(index1, worldAndGeneration, value);

  /// The collision filter of the shape.
  Filter get filter {
    final (categoryBits, maskBits, groupIndex) = rawBox2D.shapeGetFilter(
      index1,
      worldAndGeneration,
    );
    return Filter(
      categoryBits: categoryBits,
      maskBits: maskBits,
      groupIndex: groupIndex,
    );
  }

  set filter(Filter value) => rawBox2D.shapeSetFilter(
    index1,
    worldAndGeneration,
    value.categoryBits,
    value.maskBits,
    value.groupIndex,
  );

  /// Whether the shape reports sensor overlap events.
  bool get sensorEventsEnabled =>
      rawBox2D.shapeAreSensorEventsEnabled(index1, worldAndGeneration);

  set sensorEventsEnabled(bool value) => rawBox2D.shapeEnableSensorEvents(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// Whether the shape reports contact begin and end events.
  bool get contactEventsEnabled =>
      rawBox2D.shapeAreContactEventsEnabled(index1, worldAndGeneration);

  set contactEventsEnabled(bool value) => rawBox2D.shapeEnableContactEvents(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// Whether the shape reports contact hit events.
  bool get hitEventsEnabled =>
      rawBox2D.shapeAreHitEventsEnabled(index1, worldAndGeneration);

  set hitEventsEnabled(bool value) =>
      rawBox2D.shapeEnableHitEvents(index1, worldAndGeneration, enabled: value);

  /// Whether the shape reports pre-solve events.
  bool get preSolveEventsEnabled =>
      rawBox2D.shapeArePreSolveEventsEnabled(index1, worldAndGeneration);

  set preSolveEventsEnabled(bool value) => rawBox2D.shapeEnablePreSolveEvents(
    index1,
    worldAndGeneration,
    enabled: value,
  );

  /// Whether the given world point is inside the shape.
  bool testPoint(Vector2 point) =>
      rawBox2D.shapeTestPoint(index1, worldAndGeneration, point.x, point.y);

  /// The geometry of the shape in the body's local coordinates, read back
  /// from the simulation.
  ///
  /// Polygons return their convex hull, so a [Polygon.box] comes back as a
  /// four-vertex [Polygon]. Chain segments come back as the [Segment] they
  /// collide with.
  ShapeGeometry get geometry {
    switch (type) {
      case ShapeType.circle:
        final (centerX, centerY, radius) = rawBox2D.shapeGetCircle(
          index1,
          worldAndGeneration,
        );
        return Circle(center: Vector2(centerX, centerY), radius: radius);
      case ShapeType.capsule:
        final (center1X, center1Y, center2X, center2Y, radius) = rawBox2D
            .shapeGetCapsule(index1, worldAndGeneration);
        return Capsule(
          center1: Vector2(center1X, center1Y),
          center2: Vector2(center2X, center2Y),
          radius: radius,
        );
      case ShapeType.segment:
        final (point1X, point1Y, point2X, point2Y) = rawBox2D.shapeGetSegment(
          index1,
          worldAndGeneration,
        );
        return Segment(
          point1: Vector2(point1X, point1Y),
          point2: Vector2(point2X, point2Y),
        );
      case ShapeType.chainSegment:
        final (point1X, point1Y, point2X, point2Y) = rawBox2D
            .shapeGetChainSegment(index1, worldAndGeneration);
        return Segment(
          point1: Vector2(point1X, point1Y),
          point2: Vector2(point2X, point2Y),
        );
      case ShapeType.polygon:
        final polygon = rawBox2D.shapeGetPolygon(index1, worldAndGeneration);
        return Polygon(
          [
            for (var i = 0; i < polygon.points.length; i += 2)
              Vector2(polygon.points[i], polygon.points[i + 1]),
          ],
          radius: polygon.radius,
        );
    }
  }

  /// The current world axis-aligned bounding box of the shape.
  Aabb get aabb {
    final (lowerX, lowerY, upperX, upperY) = rawBox2D.shapeGetAabb(
      index1,
      worldAndGeneration,
    );
    return Aabb(Vector2(lowerX, lowerY), Vector2(upperX, upperY));
  }

  /// The user data associated with the shape.
  Object? get userData => world.shapeUserData[(index1, worldAndGeneration)];

  set userData(Object? value) {
    if (value == null) {
      world.shapeUserData.remove((index1, worldAndGeneration));
    } else {
      world.shapeUserData[(index1, worldAndGeneration)] = value;
    }
  }

  /// Destroys this shape and removes it from its body.
  ///
  /// When [updateBodyMass] is false, call [Body.applyMassFromShapes]
  /// afterwards.
  ///
  /// Safe to call while the world is stepping: the destruction is deferred
  /// until the step ends, and the shape stays valid until then.
  void destroy({bool updateBodyMass = true}) {
    if (world.locked) {
      world.deferredActions.add(() => destroy(updateBodyMass: updateBodyMass));
      return;
    }
    if (!isValid) {
      return;
    }
    world.shapeUserData.remove((index1, worldAndGeneration));
    rawBox2D.destroyShape(
      index1,
      worldAndGeneration,
      updateBodyMass: updateBodyMass,
    );
  }

  @override
  bool operator ==(Object other) =>
      other is Shape &&
      other.world == world &&
      other.index1 == index1 &&
      other.worldAndGeneration == worldAndGeneration;

  @override
  int get hashCode => Object.hash(world, index1, worldAndGeneration);

  @override
  String toString() => 'Shape($index1)';
}
