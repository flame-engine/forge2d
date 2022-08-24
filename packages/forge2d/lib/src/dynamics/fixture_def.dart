import 'package:forge2d/forge2d.dart';

/// Used to create a [Fixture].
///
/// You can reuse fixture definitions safely.
class FixtureDef {
  FixtureDef(
    this.shape, {
    this.userData,
    this.friction = 0,
    this.restitution = 0,
    this.density = 1,
    this.isSensor = false,
    Filter? filter,
  }) : filter = filter ?? Filter();

  /// The [Shape], this must be set.
  ///
  /// The [Shape] will be [Shape.clone]d, so you can create the shape on the
  /// stack.
  Shape shape;

  /// Use this to store application specific fixture data.
  Object? userData;

  /// The friction coefficient, usually in the range [0,1].
  double friction;

  /// The restitution (elasticity) usually in the range [0,1].
  double restitution;

  /// The density, usually in kg/m^2.
  double density;

  /// A sensor shape collects contact information but never generates a
  /// collision response.
  bool isSensor;

  /// Contact filtering data.
  Filter filter;
}
