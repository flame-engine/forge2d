import 'package:forge2d/src/api/body.dart';
import 'package:forge2d/src/api/debug_draw.dart';
import 'package:forge2d/src/api/defs.dart';
import 'package:forge2d/src/api/events.dart';
import 'package:forge2d/src/api/joints/distance_joint.dart';
import 'package:forge2d/src/api/joints/filter_joint.dart';
import 'package:forge2d/src/api/joints/joint.dart';
import 'package:forge2d/src/api/joints/motor_joint.dart';
import 'package:forge2d/src/api/joints/mouse_joint.dart';
import 'package:forge2d/src/api/joints/prismatic_joint.dart';
import 'package:forge2d/src/api/joints/revolute_joint.dart';
import 'package:forge2d/src/api/joints/weld_joint.dart';
import 'package:forge2d/src/api/joints/wheel_joint.dart';
import 'package:forge2d/src/api/math.dart';
import 'package:forge2d/src/api/shape.dart';
import 'package:forge2d/src/backend/raw_box2d.dart';
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
  /// setting; use [definition] for full control over the remaining `b2WorldDef`
  /// fields.
  World({Vector2? gravity, WorldDef? definition})
    : this._(gravity: gravity, definition: definition ?? WorldDef());

  World._({required WorldDef definition, Vector2? gravity})
    : id = rawBox2D.createWorld(
        gravityX: (gravity ?? definition.gravity).x,
        gravityY: (gravity ?? definition.gravity).y,
        restitutionThreshold: definition.restitutionThreshold,
        hitEventThreshold: definition.hitEventThreshold,
        contactHertz: definition.contactHertz,
        contactDampingRatio: definition.contactDampingRatio,
        maxContactPushSpeed: definition.maxContactPushSpeed,
        maximumLinearSpeed: definition.maximumLinearSpeed,
        enableSleep: definition.enableSleep,
        enableContinuous: definition.enableContinuous,
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
  Body createBody([BodyDef? definition]) {
    assert(isValid, 'World has been destroyed');
    final bodyDefinition = definition ?? BodyDef();
    final (index1, worldAndGeneration) = rawBox2D.createBody(
      id,
      type: bodyDefinition.type.index,
      positionX: bodyDefinition.position.x,
      positionY: bodyDefinition.position.y,
      rotationCos: bodyDefinition.rotation.cos,
      rotationSin: bodyDefinition.rotation.sin,
      linearVelocityX: bodyDefinition.linearVelocity.x,
      linearVelocityY: bodyDefinition.linearVelocity.y,
      angularVelocity: bodyDefinition.angularVelocity,
      linearDamping: bodyDefinition.linearDamping,
      angularDamping: bodyDefinition.angularDamping,
      gravityScale: bodyDefinition.gravityScale,
      sleepThreshold: bodyDefinition.sleepThreshold,
      name: bodyDefinition.name,
      enableSleep: bodyDefinition.enableSleep,
      isAwake: bodyDefinition.isAwake,
      fixedRotation: bodyDefinition.fixedRotation,
      isBullet: bodyDefinition.isBullet,
      isEnabled: bodyDefinition.isEnabled,
      allowFastRotation: bodyDefinition.allowFastRotation,
    );
    if (bodyDefinition.userData != null) {
      bodyUserData[(index1, worldAndGeneration)] = bodyDefinition.userData;
    }
    return Body.internal(this, index1, worldAndGeneration);
  }

  /// Creates a distance joint from [definition].
  DistanceJoint createDistanceJoint(DistanceJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createDistanceJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      localAnchorA: (definition.localAnchorA.x, definition.localAnchorA.y),
      localAnchorB: (definition.localAnchorB.x, definition.localAnchorB.y),
      length: definition.length,
      enableSpring: definition.enableSpring,
      hertz: definition.hertz,
      dampingRatio: definition.dampingRatio,
      enableLimit: definition.enableLimit,
      minLength: definition.minLength,
      maxLength: definition.maxLength,
      enableMotor: definition.enableMotor,
      maxMotorForce: definition.maxMotorForce,
      motorSpeed: definition.motorSpeed,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return DistanceJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a filter joint from [definition], disabling collision between
  /// the two bodies.
  FilterJoint createFilterJoint(FilterJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createFilterJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return FilterJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a motor joint from [definition].
  MotorJoint createMotorJoint(MotorJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createMotorJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      linearOffset: (definition.linearOffset.x, definition.linearOffset.y),
      angularOffset: definition.angularOffset,
      maxForce: definition.maxForce,
      maxTorque: definition.maxTorque,
      correctionFactor: definition.correctionFactor,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return MotorJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a mouse joint from [definition].
  MouseJoint createMouseJoint(MouseJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createMouseJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      target: (definition.target.x, definition.target.y),
      hertz: definition.hertz,
      dampingRatio: definition.dampingRatio,
      maxForce: definition.maxForce,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return MouseJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a prismatic joint from [definition].
  PrismaticJoint createPrismaticJoint(PrismaticJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createPrismaticJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      localAnchorA: (definition.localAnchorA.x, definition.localAnchorA.y),
      localAnchorB: (definition.localAnchorB.x, definition.localAnchorB.y),
      localAxisA: (definition.localAxisA.x, definition.localAxisA.y),
      referenceAngle: definition.referenceAngle,
      targetTranslation: definition.targetTranslation,
      enableSpring: definition.enableSpring,
      hertz: definition.hertz,
      dampingRatio: definition.dampingRatio,
      enableLimit: definition.enableLimit,
      lowerTranslation: definition.lowerTranslation,
      upperTranslation: definition.upperTranslation,
      enableMotor: definition.enableMotor,
      maxMotorForce: definition.maxMotorForce,
      motorSpeed: definition.motorSpeed,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return PrismaticJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a revolute joint from [definition].
  RevoluteJoint createRevoluteJoint(RevoluteJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createRevoluteJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      localAnchorA: (definition.localAnchorA.x, definition.localAnchorA.y),
      localAnchorB: (definition.localAnchorB.x, definition.localAnchorB.y),
      referenceAngle: definition.referenceAngle,
      targetAngle: definition.targetAngle,
      enableSpring: definition.enableSpring,
      hertz: definition.hertz,
      dampingRatio: definition.dampingRatio,
      enableLimit: definition.enableLimit,
      lowerAngle: definition.lowerAngle,
      upperAngle: definition.upperAngle,
      enableMotor: definition.enableMotor,
      maxMotorTorque: definition.maxMotorTorque,
      motorSpeed: definition.motorSpeed,
      drawSize: definition.drawSize,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return RevoluteJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a weld joint from [definition].
  WeldJoint createWeldJoint(WeldJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createWeldJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      localAnchorA: (definition.localAnchorA.x, definition.localAnchorA.y),
      localAnchorB: (definition.localAnchorB.x, definition.localAnchorB.y),
      referenceAngle: definition.referenceAngle,
      linearHertz: definition.linearHertz,
      angularHertz: definition.angularHertz,
      linearDampingRatio: definition.linearDampingRatio,
      angularDampingRatio: definition.angularDampingRatio,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return WeldJoint.internal(this, index1, worldAndGeneration);
  }

  /// Creates a wheel joint from [definition].
  WheelJoint createWheelJoint(WheelJointDef definition) {
    final (index1, worldAndGeneration) = rawBox2D.createWheelJoint(
      id,
      bodyA: (definition.bodyA.index1, definition.bodyA.worldAndGeneration),
      bodyB: (definition.bodyB.index1, definition.bodyB.worldAndGeneration),
      localAnchorA: (definition.localAnchorA.x, definition.localAnchorA.y),
      localAnchorB: (definition.localAnchorB.x, definition.localAnchorB.y),
      localAxisA: (definition.localAxisA.x, definition.localAxisA.y),
      enableSpring: definition.enableSpring,
      hertz: definition.hertz,
      dampingRatio: definition.dampingRatio,
      enableLimit: definition.enableLimit,
      lowerTranslation: definition.lowerTranslation,
      upperTranslation: definition.upperTranslation,
      enableMotor: definition.enableMotor,
      maxMotorTorque: definition.maxMotorTorque,
      motorSpeed: definition.motorSpeed,
      collideConnected: definition.collideConnected,
    );
    _storeJointUserData(index1, worldAndGeneration, definition);
    return WheelJoint.internal(this, index1, worldAndGeneration);
  }

  void _storeJointUserData(
    int index1,
    int worldAndGeneration,
    JointDef definition,
  ) {
    if (definition.userData != null) {
      jointUserData[(index1, worldAndGeneration)] = definition.userData;
    }
  }

  // Events. Poll these after each call to [step]; the returned collections
  // are Dart-side copies and stay usable afterwards.

  /// The contact events of the last step.
  ///
  /// Only shapes with [ShapeDef.enableContactEvents] generate begin and end
  /// events, and hit events additionally require
  /// [ShapeDef.enableHitEvents].
  ContactEvents get contactEvents {
    final events = rawBox2D.worldGetContactEvents(id);
    return ContactEvents(
      begin: [
        for (final event in events.begin)
          ContactBeginEvent(
            shapeA: Shape.internal(
              this,
              event.shapeAIndex1,
              event.shapeAWorldAndGeneration,
            ),
            shapeB: Shape.internal(
              this,
              event.shapeBIndex1,
              event.shapeBWorldAndGeneration,
            ),
            normal: Vector2(event.normalX, event.normalY),
            points: [
              for (final point in event.points)
                ContactPoint(
                  point: Vector2(point.x, point.y),
                  separation: point.separation,
                ),
            ],
          ),
      ],
      end: [
        for (final event in events.end)
          ContactEndEvent(
            shapeA: Shape.internal(
              this,
              event.shapeAIndex1,
              event.shapeAWorldAndGeneration,
            ),
            shapeB: Shape.internal(
              this,
              event.shapeBIndex1,
              event.shapeBWorldAndGeneration,
            ),
          ),
      ],
      hit: [
        for (final event in events.hit)
          ContactHitEvent(
            shapeA: Shape.internal(
              this,
              event.shapeAIndex1,
              event.shapeAWorldAndGeneration,
            ),
            shapeB: Shape.internal(
              this,
              event.shapeBIndex1,
              event.shapeBWorldAndGeneration,
            ),
            point: Vector2(event.pointX, event.pointY),
            normal: Vector2(event.normalX, event.normalY),
            approachSpeed: event.approachSpeed,
          ),
      ],
    );
  }

  /// The sensor overlap events of the last step.
  ///
  /// Sensor shapes need [ShapeDef.isSensor], and visiting shapes need
  /// [ShapeDef.enableSensorEvents].
  SensorEvents get sensorEvents {
    final events = rawBox2D.worldGetSensorEvents(id);
    return SensorEvents(
      begin: [
        for (final event in events.begin)
          SensorEvent(
            sensor: Shape.internal(
              this,
              event.sensorIndex1,
              event.sensorWorldAndGeneration,
            ),
            visitor: Shape.internal(
              this,
              event.visitorIndex1,
              event.visitorWorldAndGeneration,
            ),
          ),
      ],
      end: [
        for (final event in events.end)
          SensorEvent(
            sensor: Shape.internal(
              this,
              event.sensorIndex1,
              event.sensorWorldAndGeneration,
            ),
            visitor: Shape.internal(
              this,
              event.visitorIndex1,
              event.visitorWorldAndGeneration,
            ),
          ),
      ],
    );
  }

  /// The bodies that moved during the last step, with their new transforms.
  ///
  /// Use this to sync game objects to the simulation without touching every
  /// body.
  List<BodyMoveEvent> get bodyMoveEvents => [
    for (final event in rawBox2D.worldGetBodyEvents(id))
      BodyMoveEvent(
        body: Body.internal(
          this,
          event.bodyIndex1,
          event.bodyWorldAndGeneration,
        ),
        transform: Transform(
          Vector2(event.x, event.y),
          Rot(event.rotationCos, event.rotationSin),
        ),
        fellAsleep: event.fellAsleep,
      ),
  ];

  // Queries.

  /// Casts a ray from [origin] along [translation] and returns the closest
  /// hit, or null when nothing is hit.
  RayHit? castRayClosest(
    Vector2 origin,
    Vector2 translation, {
    QueryFilter? filter,
  }) {
    final queryFilter = filter ?? QueryFilter();
    final hit = rawBox2D.worldCastRayClosest(
      id,
      origin.x,
      origin.y,
      translation.x,
      translation.y,
      queryFilter.categoryBits,
      queryFilter.maskBits,
    );
    if (hit == null) {
      return null;
    }
    return _rayHit(hit);
  }

  /// Casts a ray from [origin] along [translation], invoking [callback] for
  /// every candidate hit in an arbitrary order.
  ///
  /// The callback controls the rest of the cast with its return value:
  /// -1 to ignore the hit, 0 to stop, the hit's fraction to clip the ray to
  /// the hit, or 1 to continue looking without clipping.
  void castRay(
    Vector2 origin,
    Vector2 translation,
    double Function(RayHit hit) callback, {
    QueryFilter? filter,
  }) {
    final queryFilter = filter ?? QueryFilter();
    rawBox2D.worldCastRay(
      id,
      origin.x,
      origin.y,
      translation.x,
      translation.y,
      queryFilter.categoryBits,
      queryFilter.maskBits,
      (hit) => callback(_rayHit(hit)),
    );
  }

  /// Casts a ray from [origin] along [translation] and returns every hit,
  /// sorted from nearest to farthest.
  List<RayHit> castRayAll(
    Vector2 origin,
    Vector2 translation, {
    QueryFilter? filter,
  }) {
    final hits = <RayHit>[];
    castRay(origin, translation, filter: filter, (hit) {
      hits.add(hit);
      return 1;
    });
    return hits..sort((a, b) => a.fraction.compareTo(b.fraction));
  }

  RayHit _rayHit(RawRayHit hit) => RayHit(
    shape: Shape.internal(this, hit.shapeIndex1, hit.shapeWorldAndGeneration),
    point: Vector2(hit.pointX, hit.pointY),
    normal: Vector2(hit.normalX, hit.normalY),
    fraction: hit.fraction,
  );

  /// Returns all shapes whose bounding boxes overlap [aabb].
  List<Shape> overlapAabb(Aabb aabb, {QueryFilter? filter}) {
    final queryFilter = filter ?? QueryFilter();
    final ids = rawBox2D.worldOverlapAabb(
      id,
      aabb.lowerBound.x,
      aabb.lowerBound.y,
      aabb.upperBound.x,
      aabb.upperBound.y,
      queryFilter.categoryBits,
      queryFilter.maskBits,
    );
    return [
      for (var i = 0; i < ids.length; i += 2)
        Shape.internal(this, ids[i], ids[i + 1]),
    ];
  }

  /// Applies a radial explosion impulse to all shapes within reach.
  void explode(ExplosionDef definition) => rawBox2D.worldExplode(
    id,
    maskBits: definition.maskBits,
    positionX: definition.position.x,
    positionY: definition.position.y,
    radius: definition.radius,
    falloff: definition.falloff,
    impulsePerLength: definition.impulsePerLength,
  );

  // Simulation callbacks.

  /// Sets or clears a custom collision filter, consulted for shape pairs
  /// that pass the regular category and group filtering. It returns whether
  /// the shapes may collide.
  ///
  /// The callback runs during [step] and must not access the world.
  set customFilterCallback(
    bool Function(Shape shapeA, Shape shapeB)? callback,
  ) {
    if (callback == null) {
      rawBox2D.worldSetCustomFilterCallback(id, null);
    } else {
      rawBox2D.worldSetCustomFilterCallback(
        id,
        (aIndex1, aWorldAndGeneration, bIndex1, bWorldAndGeneration) =>
            callback(
              Shape.internal(this, aIndex1, aWorldAndGeneration),
              Shape.internal(this, bIndex1, bWorldAndGeneration),
            ),
      );
    }
  }

  /// Sets or clears the pre-solve callback, invoked during [step] after a
  /// contact manifold is computed but before the solver runs. Returning
  /// false disables the contact for the step, enabling one-sided platforms
  /// and similar tricks.
  ///
  /// Only shapes with [ShapeDef.enablePreSolveEvents] participate. The
  /// callback must be fast and must not access the world.
  set preSolveCallback(
    bool Function(Shape shapeA, Shape shapeB, Vector2 normal)? callback,
  ) {
    if (callback == null) {
      rawBox2D.worldSetPreSolveCallback(id, null);
    } else {
      rawBox2D.worldSetPreSolveCallback(
        id,
        (
          aIndex1,
          aWorldAndGeneration,
          bIndex1,
          bWorldAndGeneration,
          normalX,
          normalY,
        ) => callback(
          Shape.internal(this, aIndex1, aWorldAndGeneration),
          Shape.internal(this, bIndex1, bWorldAndGeneration),
          Vector2(normalX, normalY),
        ),
      );
    }
  }

  /// Draws the world's debug data into [debugDraw].
  void draw(DebugDraw debugDraw) {
    List<Vector2> unflatten(List<double> vertices) => [
      for (var i = 0; i < vertices.length; i += 2)
        Vector2(vertices[i], vertices[i + 1]),
    ];
    final bounds = debugDraw.drawingBounds;
    rawBox2D.worldDraw(id, (
      drawingBounds: bounds == null
          ? null
          : (
              bounds.lowerBound.x,
              bounds.lowerBound.y,
              bounds.upperBound.x,
              bounds.upperBound.y,
            ),
      drawShapes: debugDraw.drawShapes,
      drawJoints: debugDraw.drawJoints,
      drawJointExtras: debugDraw.drawJointExtras,
      drawBounds: debugDraw.drawBounds,
      drawMass: debugDraw.drawMass,
      drawBodyNames: debugDraw.drawBodyNames,
      drawContacts: debugDraw.drawContacts,
      drawGraphColors: debugDraw.drawGraphColors,
      drawContactNormals: debugDraw.drawContactNormals,
      drawContactImpulses: debugDraw.drawContactImpulses,
      drawContactFeatures: debugDraw.drawContactFeatures,
      drawFrictionImpulses: debugDraw.drawFrictionImpulses,
      drawIslands: debugDraw.drawIslands,
      drawPolygon: (vertices, color) =>
          debugDraw.drawPolygon(unflatten(vertices), color),
      drawSolidPolygon:
          (
            positionX,
            positionY,
            rotationCos,
            rotationSin,
            vertices,
            radius,
            color,
          ) => debugDraw.drawSolidPolygon(
            Transform(
              Vector2(positionX, positionY),
              Rot(rotationCos, rotationSin),
            ),
            unflatten(vertices),
            radius,
            color,
          ),
      drawCircle: (centerX, centerY, radius, color) =>
          debugDraw.drawCircle(Vector2(centerX, centerY), radius, color),
      drawSolidCircle:
          (positionX, positionY, rotationCos, rotationSin, radius, color) =>
              debugDraw.drawSolidCircle(
                Transform(
                  Vector2(positionX, positionY),
                  Rot(rotationCos, rotationSin),
                ),
                radius,
                color,
              ),
      drawSolidCapsule: (point1X, point1Y, point2X, point2Y, radius, color) =>
          debugDraw.drawSolidCapsule(
            Vector2(point1X, point1Y),
            Vector2(point2X, point2Y),
            radius,
            color,
          ),
      drawSegment: (point1X, point1Y, point2X, point2Y, color) =>
          debugDraw.drawSegment(
            Vector2(point1X, point1Y),
            Vector2(point2X, point2Y),
            color,
          ),
      drawTransform: (positionX, positionY, rotationCos, rotationSin) =>
          debugDraw.drawTransform(
            Transform(
              Vector2(positionX, positionY),
              Rot(rotationCos, rotationSin),
            ),
          ),
      drawPoint: (x, y, size, color) =>
          debugDraw.drawPoint(Vector2(x, y), size, color),
      drawString: (x, y, text, color) =>
          debugDraw.drawString(Vector2(x, y), text, color),
    ));
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
