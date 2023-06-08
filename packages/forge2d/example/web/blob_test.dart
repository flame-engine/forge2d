import 'dart:math';

import 'package:forge2d/forge2d.dart';

import 'demo.dart';

class BlobTest extends Demo {
  /// Constructs a new BlobTest.
  BlobTest() : super('Blob test');

  /// Entrypoint.
  static void main() {
    final blob = BlobTest();
    blob.initialize();
    blob.initializeAnimation();
    blob.runAnimation();
  }

  @override
  void initialize() {
    Body ground;
    {
      final sd = PolygonShape();
      sd.setAsBoxXY(50.0, 0.4);

      final bd = BodyDef();
      bd.position.setValues(0.0, 0.0);
      ground = world.createBody(bd);
      ground.createFixtureFromShape(sd);

      sd.setAsBox(0.4, 50.0, Vector2(-10.0, 0.0), 0.0);
      ground.createFixtureFromShape(sd);
      sd.setAsBox(0.4, 50.0, Vector2(10.0, 0.0), 0.0);
      ground.createFixtureFromShape(sd);
    }

    final jointDef = ConstantVolumeJointDef();

    const cx = 0.0;
    const cy = 10.0;
    const rx = 5.0;
    const ry = 5.0;
    const nBodies = 20.0;
    const bodyRadius = 0.5;
    for (var i = 0; i < nBodies; ++i) {
      final angle = (i / nBodies) * pi * 2;
      final bd = BodyDef();
      bd.fixedRotation = true;

      final x = cx + rx * sin(angle);
      final y = cy + ry * cos(angle);
      bd.position.setFrom(Vector2(x, y));
      bd.type = BodyType.dynamic;
      final body = world.createBody(bd);

      final shape = CircleShape()..radius = bodyRadius;
      final fixtureDef = FixtureDef(shape)
        ..density = 1.0
        ..filter.groupIndex = -2;
      body.createFixture(fixtureDef);
      jointDef.addBody(body);
    }

    jointDef.frequencyHz = 10.0;
    jointDef.dampingRatio = 1.0;
    jointDef.collideConnected = false;

    final constantVolumeJoint = ConstantVolumeJoint(world, jointDef);
    world.createJoint(constantVolumeJoint);

    final bd2 = BodyDef();
    bd2.type = BodyType.dynamic;
    final psd = PolygonShape();
    psd.setAsBox(3.0, 1.5, Vector2(cx, cy + 15.0), 0.0);
    bd2.position = Vector2(cx, cy + 15.0);
    final fallingBox = world.createBody(bd2);
    fallingBox.createFixtureFromShape(psd);
  }
}

void main() {
  BlobTest.main();
}
