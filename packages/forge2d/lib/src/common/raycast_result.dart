import 'package:vector_math/vector_math.dart';

class RaycastResult {
  double lambda = 0.0;
  final Vector2 normal = Vector2.zero();

  void setFrom(RaycastResult argOther) {
    lambda = argOther.lambda;
    normal.setFrom(argOther.normal);
  }
}
