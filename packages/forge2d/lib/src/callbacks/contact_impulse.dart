import 'dart:typed_data';

import '../settings.dart' as settings;

/// Contact impulses for reporting. Impulses are used instead of forces because
/// sub-step forces may approach infinity for rigid body collisions. These match
/// up one-to-one with the contact points in b2Manifold.
class ContactImpulse {
  Float64List normalImpulses = Float64List(settings.maxManifoldPoints);
  Float64List tangentImpulses = Float64List(settings.maxManifoldPoints);
  int count = 0;
}
