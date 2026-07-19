import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';

/// The example color palette, applied through shape custom colors.
abstract final class Palette {
  static const indigo = 0x7C9CFF;
  static const sky = 0x38BDF8;
  static const amber = 0xFBBF24;
  static const rose = 0xFB7185;
  static const emerald = 0x34D399;
  static const violet = 0xA78BFA;
  static const slate = 0x64748B;

  static const dynamics = [indigo, sky, amber, rose, emerald, violet];

  /// A stable pseudo-random palette color for index [i].
  static int dynamic(int i) => dynamics[i % dynamics.length];
}

ShapeDef _solid(int color, {double? friction, double? restitution}) => ShapeDef(
  material: SurfaceMaterial(
    friction: friction ?? 0.6,
    restitution: restitution ?? 0,
    customColor: color,
  ),
);

ShapeDef _static() => ShapeDef(
  material: SurfaceMaterial(customColor: Palette.slate),
);

/// Everything a scene sets up beyond bodies: an anchor for mouse dragging,
/// and optional tick and key hooks.
class SceneHooks {
  /// A static body scenes register for mouse joints to attach to.
  Body? anchor;

  /// Called every fixed step with the step time.
  void Function(double timeStep)? onTick;

  /// Called on key down and key up, with `down` telling which.
  void Function(String key, {required bool down})? onKey;

  /// When set, the camera centers on the returned world point every frame.
  Vector2 Function()? cameraFollow;
}

/// A selectable demo scene.
class Scene {
  const Scene({
    required this.name,
    required this.hint,
    required this.viewHeight,
    required this.centerY,
    required this.build,
  });

  final String name;
  final String hint;
  final double viewHeight;
  final double centerY;
  final void Function(World world, SceneHooks hooks) build;
}

/// All demo scenes, in display order.
final scenes = <Scene>[
  Scene(
    name: 'Pyramid',
    hint: 'Drag boxes with the pointer',
    viewHeight: 26,
    centerY: 10,
    build: (world, hooks) {
      hooks.anchor = _ground(world);
      const rows = 14;
      var tint = 0;
      for (var row = 0; row < rows; row++) {
        final count = rows - row;
        final y = 0.55 + row * 1.05;
        for (var i = 0; i < count; i++) {
          final x = (i - (count - 1) / 2) * 1.1;
          world
              .createBody(
                BodyDef(type: BodyType.dynamic, position: Vector2(x, y)),
              )
              .createShape(
                Polygon.square(0.5),
                _solid(Palette.dynamic(tint++)),
              );
        }
      }
    },
  ),
  Scene(
    name: 'Domino tower',
    hint: 'A heavy ball topples the tower; drag the debris around',
    viewHeight: 30,
    centerY: 8,
    build: (world, hooks) {
      hooks.anchor = _ground(world);
      const rows = 10;
      var tint = 0;
      for (var row = 0; row < rows; row++) {
        final count = rows - row;
        final y = 0.45 + row * 0.95;
        for (var i = 0; i < count; i++) {
          final x = (i - (count - 1) / 2) * 1.1;
          world
              .createBody(
                BodyDef(type: BodyType.dynamic, position: Vector2(x, y)),
              )
              .createShape(
                Polygon.box(0.05, 0.45),
                _solid(Palette.dynamic(tint++)),
              );
        }
      }
      world
          .createBody(
            BodyDef(
              type: BodyType.dynamic,
              position: Vector2(-30, 6),
              linearVelocity: Vector2(35, 0),
              isBullet: true,
            ),
          )
          .createShape(
            Circle(radius: 0.9),
            ShapeDef(
              density: 8,
              material: SurfaceMaterial(customColor: Palette.rose),
            ),
          );
    },
  ),
  Scene(
    name: 'Ball cage',
    hint: 'A spinning paddle keeps the balls moving',
    viewHeight: 34,
    centerY: 0,
    build: (world, hooks) {
      world.gravity = Vector2.zero();
      final cage = world.createBody();
      hooks.anchor = cage;
      const cageRadius = 15.0;
      cage.createChain(
        ChainDef(
          points: [
            for (var i = 0; i < 40; i++)
              Vector2(
                cageRadius * math.cos(-2 * math.pi * i / 40),
                cageRadius * math.sin(-2 * math.pi * i / 40),
              ),
          ],
          isLoop: true,
        ),
      );

      final paddle = world.createBody(
        BodyDef(type: BodyType.kinematic, angularVelocity: 0.8),
      )..createShape(Polygon.box(11, 0.4), _solid(Palette.violet));
      hooks.onTick = (_) => paddle.isAwake = true;

      final random = math.Random(1);
      for (var i = 0; i < 60; i++) {
        final angle = random.nextDouble() * 2 * math.pi;
        final distance = random.nextDouble() * 10 + 2;
        world
            .createBody(
              BodyDef(
                type: BodyType.dynamic,
                position: Vector2(
                  distance * math.cos(angle),
                  distance * math.sin(angle),
                ),
              ),
            )
            .createShape(
              Circle(radius: 0.35 + random.nextDouble() * 0.35),
              _solid(Palette.dynamic(i), restitution: 0.7, friction: 0.1),
            );
      }
    },
  ),
  Scene(
    name: 'Circle stress',
    hint: 'Hundreds of balls, one container',
    viewHeight: 34,
    centerY: 12,
    build: (world, hooks) {
      hooks.anchor = _ground(world, width: 18);
      for (final x in [-18.0, 18.0]) {
        world
            .createBody(BodyDef(position: Vector2(x, 12)))
            .createShape(Polygon.box(0.5, 14), _static());
      }
      final random = math.Random(7);
      for (var i = 0; i < 320; i++) {
        world
            .createBody(
              BodyDef(
                type: BodyType.dynamic,
                position: Vector2(
                  random.nextDouble() * 30 - 15,
                  6 + random.nextDouble() * 20,
                ),
              ),
            )
            .createShape(
              Circle(radius: 0.25 + random.nextDouble() * 0.5),
              _solid(
                Palette.dynamic(i),
                restitution: 0.35,
                friction: 0.2,
              ),
            );
      }
    },
  ),
  Scene(
    name: 'Blob',
    hint: 'A soft body of springs; drag it around',
    viewHeight: 26,
    centerY: 8,
    build: (world, hooks) {
      hooks.anchor = _ground(world);
      const segments = 24;
      const blobRadius = 3.5;
      final center = Vector2(0, 12);
      final ring = <Body>[
        for (var i = 0; i < segments; i++)
          world.createBody(
            BodyDef(
              type: BodyType.dynamic,
              position:
                  center +
                  Vector2(
                    blobRadius * math.cos(2 * math.pi * i / segments),
                    blobRadius * math.sin(2 * math.pi * i / segments),
                  ),
              fixedRotation: true,
            ),
          )..createShape(
            Circle(radius: 0.6),
            _solid(Palette.emerald, friction: 0.9),
          ),
      ];
      DistanceJointDef spring(Body a, Body b) => DistanceJointDef(
        bodyA: a,
        bodyB: b,
        length: (a.position - b.position).length,
        enableSpring: true,
        hertz: 6,
        dampingRatio: 0.4,
      );
      for (var i = 0; i < segments; i++) {
        world
          ..createDistanceJoint(spring(ring[i], ring[(i + 1) % segments]))
          ..createDistanceJoint(
            spring(ring[i], ring[(i + segments ~/ 2) % segments]),
          );
      }
    },
  ),
  Scene(
    name: 'Bridge',
    hint: 'A plank bridge under load',
    viewHeight: 24,
    centerY: 6,
    build: (world, hooks) {
      final left = world.createBody(BodyDef(position: Vector2(-14, 6)))
        ..createShape(Polygon.box(0.6, 6), _static());
      final right = world.createBody(BodyDef(position: Vector2(14, 6)))
        ..createShape(Polygon.box(0.6, 6), _static());
      hooks.anchor = left;

      const planks = 20;
      var previous = left;
      var anchorOnPrevious = Vector2(0.6, 5.6);
      for (var i = 0; i < planks; i++) {
        final x = -13.4 + 26.8 * (i + 0.5) / planks;
        final plank =
            world.createBody(
              BodyDef(type: BodyType.dynamic, position: Vector2(x, 11.6)),
            )..createShape(
              Polygon.box(26.8 / planks / 2 - 0.05, 0.15),
              _solid(Palette.amber, friction: 0.8),
            );
        world.createRevoluteJoint(
          RevoluteJointDef(
            bodyA: previous,
            bodyB: plank,
            localAnchorA: anchorOnPrevious,
            localAnchorB: Vector2(-26.8 / planks / 2, 0),
          ),
        );
        previous = plank;
        anchorOnPrevious = Vector2(26.8 / planks / 2, 0);
      }
      world.createRevoluteJoint(
        RevoluteJointDef(
          bodyA: previous,
          bodyB: right,
          localAnchorA: anchorOnPrevious,
          localAnchorB: Vector2(-0.6, 5.6),
        ),
      );

      final random = math.Random(3);
      for (var i = 0; i < 8; i++) {
        world
            .createBody(
              BodyDef(
                type: BodyType.dynamic,
                position: Vector2(random.nextDouble() * 16 - 8, 16 + i * 1.5),
              ),
            )
            .createShape(
              i.isEven
                  ? Circle(radius: 0.7)
                  : Polygon.square(0.6) as ShapeGeometry,
              _solid(Palette.dynamic(i), friction: 0.4),
            );
      }
    },
  ),
  Scene(
    name: 'Racer',
    hint: 'Drive with the left and right arrow keys',
    viewHeight: 24,
    centerY: 3,
    build: (world, hooks) {
      // Rolling terrain; chains are one-sided, so the points run right to
      // left for ground driven on from above.
      final ground = world.createBody();
      hooks.anchor = ground;
      ground.createChain(
        ChainDef(
          points: [
            for (var x = 400.0; x >= -30.0; x -= 2)
              Vector2(x, math.sin(x / 9) * 1.8 + x * 0.015),
          ],
        ),
      );

      final chassis = world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(0, 4)),
      )..createShape(Polygon.box(1.4, 0.35), _solid(Palette.indigo));

      Body wheel(double x) =>
          world.createBody(
            BodyDef(type: BodyType.dynamic, position: Vector2(x, 3.5)),
          )..createShape(
            Circle(radius: 0.45),
            ShapeDef(
              material: SurfaceMaterial(
                friction: 1.6,
                customColor: Palette.rose,
              ),
            ),
          );

      final joints = <WheelJoint>[
        for (final x in [-1.0, 1.0])
          world.createWheelJoint(
            WheelJointDef(
              bodyA: chassis,
              bodyB: wheel(x),
              localAnchorA: Vector2(x, -0.35),
              hertz: 4,
              enableMotor: true,
              maxMotorTorque: 40,
            ),
          ),
      ];

      var throttle = 0.0;
      hooks
        ..cameraFollow = (() => chassis.position + Vector2(0, 3))
        ..onKey = (key, {required bool down}) {
          if (key == 'ArrowRight') {
            throttle = down ? -30 : 0;
          } else if (key == 'ArrowLeft') {
            throttle = down ? 30 : 0;
          }
        }
        ..onTick = (_) {
          for (final joint in joints) {
            joint.motorSpeed = throttle;
          }
        };
    },
  ),
];

Body _ground(World world, {double width = 40}) =>
    world.createBody(BodyDef(position: Vector2(0, -1)))
      ..createShape(Polygon.box(width, 1), _static());
