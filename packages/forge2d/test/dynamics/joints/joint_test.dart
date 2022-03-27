import 'package:forge2d/forge2d.dart';
import 'package:test/test.dart';

class UnknownJointDef extends JointDef {}

class UnknownJoint extends Joint {
  UnknownJoint() : super(UnknownJointDef());

  @override
  void noSuchMethod(_) {}
}

void main() {
  group('Joint', () {
    test('destruction of body with joint', () {
      final world = World(Vector2(0.0, -10.0));
      final bodyDef = BodyDef();
      final body1 = world.createBody(bodyDef);
      final body2 = world.createBody(bodyDef..position = Vector2.all(2));
      final shape = CircleShape()
        ..radius = 1.2
        ..position.setValues(10, 10);

      final fixtureDef = FixtureDef(shape)
        ..density = 50.0
        ..friction = .1
        ..restitution = .9;

      body1.createFixture(fixtureDef);
      body2.createFixture(fixtureDef);

      final revoluteJointDef = RevoluteJointDef()
        ..initialize(body1, body2, body1.position);
      final revoluteJoint = RevoluteJoint(revoluteJointDef);
      world.createJoint(revoluteJoint);

      expect(body1.joints.length, 1);
      expect(body2.joints.length, 1);
      world.destroyBody(body1);
      expect(body1.joints.length, 0);
      expect(body2.joints.length, 0);
    });
  });
}
