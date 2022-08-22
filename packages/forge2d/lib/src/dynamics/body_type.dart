import 'package:forge2d/forge2d.dart';

/// Defines the type of a [Body].
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
