library blob_test;

import 'dart:math' as math;
import 'demo.dart';
import 'package:forge2d/forge2d.dart';

class BlobTest extends Demo {
  /// Constructs a new BlobTest.
  BlobTest() : super("Blob test");

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
      final PolygonShape sd = PolygonShape();
      sd.setAsBoxXY(50.0, 0.4);

      final BodyDef bd = BodyDef();
      bd.position.setValues(0.0, 0.0);
      ground = world.createBody(bd);
      ground.createFixtureFromShape(sd);

      sd.setAsBox(0.4, 50.0, Vector2(-10.0, 0.0), 0.0);
      ground.createFixtureFromShape(sd);
      sd.setAsBox(0.4, 50.0, Vector2(10.0, 0.0), 0.0);
      ground.createFixtureFromShape(sd);
    }

    final ConstantVolumeJointDef jointDef = ConstantVolumeJointDef();

    const double cx = 0.0;
    const double cy = 10.0;
    const double rx = 5.0;
    const double ry = 5.0;
    const double nBodies = 20.0;
    const double bodyRadius = 0.5;
    for (int i = 0; i < nBodies; ++i) {
      final double angle = (i / nBodies) * math.pi * 2;
      final BodyDef bd = BodyDef();
      bd.fixedRotation = true;

      final double x = cx + rx * math.sin(angle);
      final double y = cy + ry * math.cos(angle);
      bd.position.setFrom(Vector2(x, y));
      bd.type = BodyType.DYNAMIC;
      final Body body = world.createBody(bd);

      final FixtureDef fd = FixtureDef();
      final CircleShape cd = CircleShape();
      cd.radius = bodyRadius;
      fd.shape = cd;
      fd.density = 1.0;
      fd.filter.groupIndex = -2;
      body.createFixture(fd);
      jointDef.addBody(body);
    }

    jointDef.frequencyHz = 10.0;
    jointDef.dampingRatio = 1.0;
    jointDef.collideConnected = false;
    world.createJoint(jointDef);

    final BodyDef bd2 = BodyDef();
    bd2.type = BodyType.DYNAMIC;
    final PolygonShape psd = PolygonShape();
    psd.setAsBox(3.0, 1.5, Vector2(cx, cy + 15.0), 0.0);
    bd2.position = Vector2(cx, cy + 15.0);
    final Body fallingBox = world.createBody(bd2);
    fallingBox.createFixtureFromShape(psd, 1.0);
  }
}

void main() {
  BlobTest.main();
}
