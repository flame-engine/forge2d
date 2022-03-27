import '../../forge2d.dart';

/// {@template dynamics.body_type}
/// Defines the type of a [Body].
/// {@endtemplate}
enum BodyType {
  /// Defines a [Body] with zero mass, zero velocity, may be manually moved.
  static,

  /// Defines a [Body] with zero mass, non-zero velocity set by user, moved by
  /// solver.
  kinematic,

  /// Defines a [Body] with positive mass, non-zero velocity determined by
  /// forces, moved by solver.
  dynamic,
}
