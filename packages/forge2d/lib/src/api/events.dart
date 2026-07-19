import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/math.dart';
import 'package:forge2d/src/api/shape.dart';
import 'package:vector_math/vector_math.dart';

/// A contact point of a begin-touch event's manifold.
class ContactPoint {
  /// Creates a contact point.
  ContactPoint({required this.point, required this.separation});

  /// The contact position in world coordinates.
  final Vector2 point;

  /// The separation of the surfaces, negative when overlapping.
  final double separation;
}

/// Two shapes began touching.
class ContactBeginEvent {
  /// Creates a begin-touch event.
  ContactBeginEvent({
    required this.shapeA,
    required this.shapeB,
    required this.normal,
    required this.points,
  });

  /// The first shape.
  final Shape shapeA;

  /// The second shape.
  final Shape shapeB;

  /// The contact normal in world coordinates, pointing from [shapeA] to
  /// [shapeB].
  final Vector2 normal;

  /// The initial contact points, recorded before the solver ran.
  final List<ContactPoint> points;
}

/// Two shapes stopped touching.
///
/// The shapes may already have been destroyed; check [Shape.isValid] before
/// using them.
class ContactEndEvent {
  /// Creates an end-touch event.
  ContactEndEvent({required this.shapeA, required this.shapeB});

  /// The first shape.
  final Shape shapeA;

  /// The second shape.
  final Shape shapeB;
}

/// Two shapes hit each other faster than the world's hit event threshold.
class ContactHitEvent {
  /// Creates a hit event.
  ContactHitEvent({
    required this.shapeA,
    required this.shapeB,
    required this.point,
    required this.normal,
    required this.approachSpeed,
  });

  /// The first shape.
  final Shape shapeA;

  /// The second shape.
  final Shape shapeB;

  /// The impact point in world coordinates.
  final Vector2 point;

  /// The impact normal, pointing from [shapeA] to [shapeB].
  final Vector2 normal;

  /// The closing speed of the shapes at the impact point, in meters per
  /// second.
  final double approachSpeed;
}

/// The contact events of the last step.
class ContactEvents {
  /// Creates the event collection.
  ContactEvents({required this.begin, required this.end, required this.hit});

  /// The shapes that began touching.
  final List<ContactBeginEvent> begin;

  /// The shapes that stopped touching.
  final List<ContactEndEvent> end;

  /// The impacts above the world's hit event threshold.
  final List<ContactHitEvent> hit;
}

/// A shape began or stopped overlapping a sensor shape.
///
/// On end events the shapes may already have been destroyed; check
/// [Shape.isValid] before using them.
class SensorEvent {
  /// Creates a sensor event.
  SensorEvent({required this.sensor, required this.visitor});

  /// The sensor shape.
  final Shape sensor;

  /// The shape that entered or left the sensor.
  final Shape visitor;
}

/// The sensor events of the last step.
class SensorEvents {
  /// Creates the event collection.
  SensorEvents({required this.begin, required this.end});

  /// The overlaps that started.
  final List<SensorEvent> begin;

  /// The overlaps that ended.
  final List<SensorEvent> end;
}

/// A body moved during the last step.
class BodyMoveEvent {
  /// Creates a body move event.
  BodyMoveEvent({
    required this.body,
    required this.transform,
    required this.fellAsleep,
  });

  /// The body that moved.
  final Body body;

  /// The body's new transform.
  final Transform transform;

  /// Whether the body fell asleep during the step.
  final bool fellAsleep;
}

/// A ray cast hit.
class RayHit {
  /// Creates a ray hit.
  RayHit({
    required this.shape,
    required this.point,
    required this.normal,
    required this.fraction,
  });

  /// The shape that was hit.
  final Shape shape;

  /// The hit point in world coordinates.
  final Vector2 point;

  /// The surface normal at the hit point.
  final Vector2 normal;

  /// The fraction of the ray translation at the hit point, in `[0, 1]`.
  final double fraction;
}
