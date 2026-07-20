// A port of the classic forge2d car demo: a box body on two wheel joints
// driving over hilly chain terrain, printing telemetry as it goes.
// Run with: dart run bin/car.dart
// ignore_for_file: avoid_print
import 'dart:math' as math;

import 'package:forge2d/forge2d.dart';

Future<void> main() async {
  await initializeForge2D();

  final world = World();

  // Hilly terrain built from a chain shape. Chains are one-sided: the solid
  // surface is to the right of the winding direction, so ground driven on
  // from above is listed right to left.
  final terrain = [
    for (var x = 200.0; x >= -10.0; x -= 2) Vector2(x, math.sin(x / 10) * 1.5),
  ];
  world.createBody().createChain(ChainDef(points: terrain));

  // The chassis with two wheels.
  final chassis = world.createBody(
    BodyDef(type: BodyType.dynamic, position: Vector2(0, 3)),
  )..createShape(Polygon.box(1.2, 0.3));

  Body wheel(double x) =>
      world.createBody(
        BodyDef(type: BodyType.dynamic, position: Vector2(x, 2.6)),
      )..createShape(
        Circle(radius: 0.4),
        ShapeDef(material: SurfaceMaterial(friction: 1.5)),
      );

  final backWheel = wheel(-0.9);
  final frontWheel = wheel(0.9);

  WheelJoint suspension(Body wheelBody) => world.createWheelJoint(
    WheelJointDef(
      bodyA: chassis,
      bodyB: wheelBody,
      localAnchorA: chassis.localPoint(wheelBody.position),
      hertz: 4,
      enableMotor: true,
      maxMotorTorque: 30,
      motorSpeed: -25,
    ),
  );

  suspension(backWheel);
  suspension(frontWheel);

  for (var second = 0; second <= 10; second++) {
    if (second > 0) {
      for (var i = 0; i < 60; i++) {
        world.step(1 / 60);
      }
    }
    final position = chassis.position;
    final speed = chassis.linearVelocity.x;
    print(
      't=${second.toString().padLeft(2)}s  '
      'x=${position.x.toStringAsFixed(1).padLeft(6)}  '
      'y=${position.y.toStringAsFixed(2).padLeft(5)}  '
      'speed=${speed.toStringAsFixed(1)} m/s',
    );
  }

  world.destroy();
}
