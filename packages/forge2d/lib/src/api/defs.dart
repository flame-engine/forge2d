import 'package:forge2d/src/api/enums.dart';
import 'package:forge2d/src/api/math.dart';
import 'package:vector_math/vector_math.dart';

/// Collision filtering data for a shape.
///
/// Mirrors the native `b2Filter` type. Category and mask bits are 64-bit
/// bit fields.
class Filter {
  /// Creates a collision filter.
  Filter({
    this.categoryBits = 1,
    this.maskBits = allCategories,
    this.groupIndex = 0,
  });

  /// A mask value with all 64 category bits set.
  static const int allCategories = -1;

  /// The collision category bits of the shape. Normally a single bit is set.
  int categoryBits;

  /// The categories this shape collides with.
  int maskBits;

  /// Collision override: shapes in the same non-zero group always collide
  /// (positive) or never collide (negative), regardless of category masks.
  int groupIndex;
}

/// Collision filtering data for queries, such as ray casts.
///
/// Mirrors the native `b2QueryFilter` type.
class QueryFilter {
  /// Creates a query filter.
  QueryFilter({this.categoryBits = 1, this.maskBits = Filter.allCategories});

  /// The collision category bits of the query.
  int categoryBits;

  /// The categories the query hits.
  int maskBits;
}

/// The surface properties of a shape.
///
/// Mirrors the native `b2SurfaceMaterial` type.
class SurfaceMaterial {
  /// Creates a surface material.
  SurfaceMaterial({
    this.friction = 0.6,
    this.restitution = 0,
    this.rollingResistance = 0,
    this.tangentSpeed = 0,
    this.userMaterialId = 0,
    this.customColor = 0,
  });

  /// The Coulomb (dry) friction coefficient, usually in the range `[0, 1]`.
  double friction;

  /// The coefficient of restitution (bounciness), usually in `[0, 1]`.
  double restitution;

  /// The rolling resistance, usually in `[0, 1]`.
  double rollingResistance;

  /// The tangent speed for conveyor belts, in meters per second.
  double tangentSpeed;

  /// A user-defined material identifier, passed with query results and in
  /// events. Not used internally.
  int userMaterialId;

  /// A custom debug-draw color in 0xRRGGBB format, or zero for the default.
  int customColor;
}

/// The data needed to construct a world.
///
/// Mirrors the native `b2WorldDef` type, with the native defaults.
class WorldDef {
  /// Creates a world definition.
  WorldDef({
    Vector2? gravity,
    this.restitutionThreshold = 1,
    this.hitEventThreshold = 1,
    this.contactHertz = 30,
    this.contactDampingRatio = 10,
    this.maxContactPushSpeed = 3,
    this.maximumLinearSpeed = 400,
    this.enableSleep = true,
    this.enableContinuous = true,
  }) : gravity = gravity ?? Vector2(0, -10);

  /// The gravity vector, in meters per second squared.
  Vector2 gravity;

  /// The relative velocity below which collisions stop being resolved with
  /// restitution, in meters per second.
  double restitutionThreshold;

  /// The relative velocity above which contact hit events are reported, in
  /// meters per second.
  double hitEventThreshold;

  /// The contact stiffness in hertz. Cycles per second.
  double contactHertz;

  /// The contact bounciness. Non-dimensional.
  double contactDampingRatio;

  /// The maximum speed at which overlapping shapes are pushed apart, in
  /// meters per second.
  double maxContactPushSpeed;

  /// The maximum linear speed of any body, in meters per second.
  double maximumLinearSpeed;

  /// Whether bodies are allowed to fall asleep when they come to rest.
  bool enableSleep;

  /// Whether continuous collision detection is enabled for fast bodies.
  bool enableContinuous;
}

/// The data needed to construct a rigid body.
///
/// Mirrors the native `b2BodyDef` type, with the native defaults.
class BodyDef {
  /// Creates a body definition.
  BodyDef({
    this.type = BodyType.static,
    Vector2? position,
    this.rotation = const Rot.identity(),
    Vector2? linearVelocity,
    this.angularVelocity = 0,
    this.linearDamping = 0,
    this.angularDamping = 0,
    this.gravityScale = 1,
    this.sleepThreshold = 0.05,
    this.name,
    this.userData,
    this.enableSleep = true,
    this.isAwake = true,
    this.fixedRotation = false,
    this.isBullet = false,
    this.isEnabled = true,
    this.allowFastRotation = false,
  }) : position = position ?? Vector2.zero(),
       linearVelocity = linearVelocity ?? Vector2.zero();

  /// The body type: static, kinematic, or dynamic.
  BodyType type;

  /// The initial world position of the body.
  ///
  /// Bodies should be created at their target position: creating a body at
  /// the origin and moving it afterwards is significantly slower.
  Vector2 position;

  /// The initial world rotation of the body.
  Rot rotation;

  /// The initial linear velocity of the body origin, in meters per second.
  Vector2 linearVelocity;

  /// The initial angular velocity of the body, in radians per second.
  double angularVelocity;

  /// Damping applied to the linear velocity. Usually left at zero.
  double linearDamping;

  /// Damping applied to the angular velocity. Usually left at zero.
  double angularDamping;

  /// A scale factor applied to the world gravity for this body.
  double gravityScale;

  /// The speed below which the body may fall asleep, in meters per second.
  double sleepThreshold;

  /// An optional name for debugging.
  String? name;

  /// An arbitrary object associated with the body.
  ///
  /// Stored on the Dart side; the native user data pointer is not used.
  Object? userData;

  /// Whether this body is allowed to fall asleep.
  bool enableSleep;

  /// Whether the body starts out awake.
  bool isAwake;

  /// Whether the body is prevented from rotating.
  bool fixedRotation;

  /// Whether continuous collision detection is used against static and
  /// kinematic bodies to prevent tunneling at high speeds.
  bool isBullet;

  /// Whether the body starts out enabled.
  bool isEnabled;

  /// Whether the body is allowed to rotate faster than 0.25 revolutions per
  /// sub-step. Enabling this can hurt stacking stability.
  bool allowFastRotation;
}

/// The data needed to construct a shape on a body.
///
/// Mirrors the native `b2ShapeDef` type, with the native defaults.
class ShapeDef {
  /// Creates a shape definition.
  ShapeDef({
    SurfaceMaterial? material,
    this.density = 1,
    Filter? filter,
    this.userData,
    this.isSensor = false,
    this.enableSensorEvents = false,
    this.enableContactEvents = false,
    this.enableHitEvents = false,
    this.enablePreSolveEvents = false,
    this.invokeContactCreation = true,
    this.updateBodyMass = true,
  }) : material = material ?? SurfaceMaterial(),
       filter = filter ?? Filter();

  /// The surface material of the shape.
  SurfaceMaterial material;

  /// The density of the shape, in kilograms per square meter.
  double density;

  /// The collision filter of the shape.
  Filter filter;

  /// An arbitrary object associated with the shape.
  ///
  /// Stored on the Dart side; the native user data pointer is not used.
  Object? userData;

  /// Whether the shape is a sensor: it detects overlap but produces no
  /// collision response.
  bool isSensor;

  /// Whether the shape generates sensor overlap events.
  bool enableSensorEvents;

  /// Whether the shape generates contact begin and end events.
  bool enableContactEvents;

  /// Whether the shape generates contact hit events.
  bool enableHitEvents;

  /// Whether the shape generates pre-solve events. Expensive; only enable
  /// when needed.
  bool enablePreSolveEvents;

  /// Whether contacts are created immediately when the shape is added to a
  /// body that may already be overlapping others. Usually left enabled.
  bool invokeContactCreation;

  /// Whether the body's mass is recomputed when the shape is added.
  bool updateBodyMass;
}

/// The data needed to construct a chain of line segments.
///
/// Chains are one-sided, designed for static level geometry.
/// Mirrors the native `b2ChainDef` type, with the native defaults.
class ChainDef {
  /// Creates a chain definition from [points].
  ChainDef({
    required this.points,
    List<SurfaceMaterial>? materials,
    Filter? filter,
    this.userData,
    this.isLoop = false,
    this.enableSensorEvents = false,
  }) : materials = materials ?? [SurfaceMaterial()],
       filter = filter ?? Filter();

  /// The chain vertices in local coordinates.
  ///
  /// For loops, the first and last point must not repeat; the chain is
  /// closed automatically when [isLoop] is true.
  List<Vector2> points;

  /// The surface materials of the chain: either a single material for the
  /// whole chain, or one material per segment.
  List<SurfaceMaterial> materials;

  /// The collision filter of the chain.
  Filter filter;

  /// An arbitrary object associated with the chain.
  ///
  /// Stored on the Dart side; the native user data pointer is not used.
  Object? userData;

  /// Whether the chain forms a closed loop.
  bool isLoop;

  /// Whether the chain segments generate sensor overlap events.
  bool enableSensorEvents;
}

/// The data needed to apply an explosion to a world.
///
/// Mirrors the native `b2ExplosionDef` type.
class ExplosionDef {
  /// Creates an explosion definition centered on [position].
  ExplosionDef({
    required this.position,
    required this.radius,
    required this.impulsePerLength,
    this.falloff = 0,
    this.maskBits = Filter.allCategories,
  });

  /// The categories of shapes affected by the explosion.
  int maskBits;

  /// The center of the explosion in world coordinates.
  Vector2 position;

  /// The radius of the explosion.
  double radius;

  /// The distance beyond the radius over which the impulse falls off to
  /// zero.
  double falloff;

  /// The impulse applied per unit length of shape surface, in
  /// kilogram-meters per second.
  double impulsePerLength;
}
