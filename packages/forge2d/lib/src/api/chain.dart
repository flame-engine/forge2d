import 'package:forge2d/src/api/shape.dart';
import 'package:forge2d/src/api/world.dart';
import 'package:forge2d/src/initialize.dart';
import 'package:meta/meta.dart';

/// A chain of one-sided line segments attached to a body, designed for
/// static level geometry.
///
/// Wraps a native `b2ChainId`. A [Chain] instance is a cheap value-like
/// handle: two handles to the same chain compare equal, and holding one
/// does not keep the chain alive.
@immutable
class Chain {
  /// Wraps an existing native chain id. Internal to forge2d.
  @internal
  const Chain.internal(this.world, this.index1, this.worldAndGeneration);

  /// The world this chain belongs to.
  final World world;

  /// The first half of the packed native chain id.
  @internal
  final int index1;

  /// The second half of the packed native chain id.
  @internal
  final int worldAndGeneration;

  /// Whether this chain has not been destroyed.
  bool get isValid => rawBox2D.chainIsValid(index1, worldAndGeneration);

  /// The friction of the chain segments.
  double get friction => rawBox2D.chainGetFriction(index1, worldAndGeneration);

  set friction(double value) =>
      rawBox2D.chainSetFriction(index1, worldAndGeneration, value);

  /// The restitution (bounciness) of the chain segments.
  double get restitution =>
      rawBox2D.chainGetRestitution(index1, worldAndGeneration);

  set restitution(double value) =>
      rawBox2D.chainSetRestitution(index1, worldAndGeneration, value);

  /// The segment shapes owned by this chain.
  List<Shape> get segments {
    final ids = rawBox2D.chainGetSegments(index1, worldAndGeneration);
    return [
      for (var i = 0; i < ids.length; i += 2)
        Shape.internal(world, ids[i], ids[i + 1]),
    ];
  }

  /// The user data associated with the chain.
  Object? get userData => world.chainUserData[(index1, worldAndGeneration)];

  set userData(Object? value) {
    if (value == null) {
      world.chainUserData.remove((index1, worldAndGeneration));
    } else {
      world.chainUserData[(index1, worldAndGeneration)] = value;
    }
  }

  /// Destroys this chain and its segment shapes.
  ///
  /// Safe to call while the world is stepping: the destruction is deferred
  /// until the step ends, and the chain stays valid until then.
  void destroy() {
    if (world.locked) {
      world.deferredActions.add(destroy);
      return;
    }
    if (!isValid) {
      return;
    }
    world.chainUserData.remove((index1, worldAndGeneration));
    world.chainOwners.remove((index1, worldAndGeneration));
    rawBox2D.destroyChain(index1, worldAndGeneration);
  }

  @override
  bool operator ==(Object other) =>
      other is Chain &&
      other.world == world &&
      other.index1 == index1 &&
      other.worldAndGeneration == worldAndGeneration;

  @override
  int get hashCode => Object.hash(world, index1, worldAndGeneration);

  @override
  String toString() => 'Chain($index1)';
}
