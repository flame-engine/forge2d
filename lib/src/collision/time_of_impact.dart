import 'dart:math';

import '../../forge2d.dart';
import '../settings.dart' as settings;

/// Input parameters for TOI
class TOIInput {
  final DistanceProxy proxyA = DistanceProxy();
  final DistanceProxy proxyB = DistanceProxy();
  final Sweep sweepA = Sweep();
  final Sweep sweepB = Sweep();

  /// defines sweep interval [0, tMax]
  double tMax = 0.0;
}

enum TOIOutputState { unknown, failed, overlapped, touching, separated }

/// Output parameters for TimeOfImpact
class TOIOutput {
  TOIOutputState state = TOIOutputState.unknown;
  double t = 0.0;
}

/// Class used for computing the time of impact. This class should not be constructed usually, just
/// retrieve from the {@link SingletonPool#getTOI()}.
class TimeOfImpact {
  static const int maxIterations = 20;
  static const int maxRootIterations = 50;

  static int toiCalls = 0;
  static int toiIterations = 0;
  static int toiMaxIterations = 0;
  static int toiRootIterations = 0;
  static int toiMaxRootIterations = 0;

  // djm pooling
  final SimplexCache _cache = SimplexCache();
  final DistanceInput _distanceInput = DistanceInput();
  final Transform _xfA = Transform.zero();
  final Transform _xfB = Transform.zero();
  final DistanceOutput _distanceOutput = DistanceOutput();
  final SeparationFunction _fcn = SeparationFunction();
  final List<int> _indexes = List<int>.filled(2, 0);
  final Sweep _sweepA = Sweep();
  final Sweep _sweepB = Sweep();

  TimeOfImpact();

  /// Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
  /// between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
  /// non-tunneling collision. If you change the time interval, you should call this function again.
  /// Note: use Distance to compute the contact point and normal at the time of impact.
  void timeOfImpact(TOIOutput output, TOIInput input) {
    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.

    ++toiCalls;

    output.state = TOIOutputState.unknown;
    output.t = input.tMax;

    final proxyA = input.proxyA;
    final proxyB = input.proxyB;

    _sweepA.set(input.sweepA);
    _sweepB.set(input.sweepB);

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    _sweepA.normalize();
    _sweepB.normalize();

    final tMax = input.tMax;

    final totalRadius = proxyA.radius + proxyB.radius;
    // djm: whats with all these constants?
    final target =
        max(settings.linearSlop, totalRadius - 3.0 * settings.linearSlop);
    final tolerance = 0.25 * settings.linearSlop;

    assert(target > tolerance);

    var t1 = 0.0;
    var iteration = 0;

    _cache.count = 0;
    _distanceInput.proxyA = input.proxyA;
    _distanceInput.proxyB = input.proxyB;
    _distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
      _sweepA.getTransform(_xfA, t1);
      _sweepB.getTransform(_xfB, t1);
      // Get the distance between shapes. We can also use the results
      // to get a separating axis
      _distanceInput.transformA = _xfA;
      _distanceInput.transformB = _xfB;
      World.distance.compute(_distanceOutput, _cache, _distanceInput);

      // If the shapes are overlapped, we give up on continuous collision.
      if (_distanceOutput.distance <= 0.0) {
        // Failure!
        output.state = TOIOutputState.overlapped;
        output.t = 0.0;
        break;
      }

      if (_distanceOutput.distance < target + tolerance) {
        // Victory!
        output.state = TOIOutputState.touching;
        output.t = t1;
        break;
      }

      // Initialize the separating axis.
      _fcn.initialize(_cache, proxyA, _sweepA, proxyB, _sweepB, t1);

      // Compute the TOI on the separating axis. We do this by successively
      // resolving the deepest point. This loop is bounded by the number of
      // vertices.
      var done = false;
      var t2 = tMax;
      var pushBackIteration = 0;
      for (;;) {
        // Find the deepest point at t2. Store the witness point indices.
        var s2 = _fcn.findMinSeparation(_indexes, t2);
        // Is the final configuration separated?
        if (s2 > target + tolerance) {
          // Victory!
          output.state = TOIOutputState.separated;
          output.t = tMax;
          done = true;
          break;
        }

        // Has the separation reached tolerance?
        if (s2 > target - tolerance) {
          // Advance the sweeps
          t1 = t2;
          break;
        }

        // Compute the initial separation of the witness points.
        var s1 = _fcn.evaluate(_indexes[0], _indexes[1], t1);
        // Check for initial overlap. This might happen if the root finder
        // runs out of iterations.
        if (s1 < target - tolerance) {
          output.state = TOIOutputState.failed;
          output.t = t1;
          done = true;
          break;
        }

        // Check for touching
        if (s1 <= target + tolerance) {
          // Victory! t1 should hold the TOI (could be 0.0).
          output.state = TOIOutputState.touching;
          output.t = t1;
          done = true;
          break;
        }

        // Compute 1D root of: f(x) - target = 0
        var rootIterationCount = 0;
        var a1 = t1;
        var a2 = t2;
        for (;;) {
          // Use a mix of the secant rule and bisection.
          double t;
          if ((rootIterationCount & 1) == 1) {
            // Secant rule to improve convergence.
            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
          } else {
            // Bisection to guarantee progress.
            t = 0.5 * (a1 + a2);
          }

          ++rootIterationCount;
          ++toiRootIterations;

          final s = _fcn.evaluate(_indexes[0], _indexes[1], t);

          if ((s - target).abs() < tolerance) {
            // t2 holds a tentative value for t1
            t2 = t;
            break;
          }

          // Ensure we continue to bracket the root.
          if (s > target) {
            a1 = t;
            s1 = s;
          } else {
            a2 = t;
            s2 = s;
          }

          if (rootIterationCount == maxRootIterations) {
            break;
          }
        }

        toiMaxRootIterations = max(toiMaxRootIterations, rootIterationCount);

        ++pushBackIteration;

        if (pushBackIteration == settings.maxPolygonVertices ||
            rootIterationCount == maxRootIterations) {
          break;
        }
      }

      ++iteration;
      ++toiIterations;

      if (done) {
        break;
      }

      if (iteration == maxIterations) {
        // Root finder got stuck. Semi-victory.
        output.state = TOIOutputState.failed;
        output.t = t1;
        break;
      }
    }

    toiMaxIterations = max(toiMaxIterations, iteration);
  }
} // Class TimeOfImpact.

enum SeparationFunctionType { points, faceA, faceB }

class SeparationFunction {
  DistanceProxy proxyA;
  DistanceProxy proxyB;
  SeparationFunctionType type;
  final Vector2 localPoint = Vector2.zero();
  final Vector2 axis = Vector2.zero();
  Sweep sweepA;
  Sweep sweepB;

  // djm pooling
  final Vector2 _localPointA = Vector2.zero();
  final Vector2 _localPointB = Vector2.zero();
  final Vector2 _pointA = Vector2.zero();
  final Vector2 _pointB = Vector2.zero();
  final Vector2 _localPointA1 = Vector2.zero();
  final Vector2 _localPointA2 = Vector2.zero();
  final Vector2 _normal = Vector2.zero();
  final Vector2 _localPointB1 = Vector2.zero();
  final Vector2 _localPointB2 = Vector2.zero();
  final Vector2 _temp = Vector2.zero();
  final Transform _xfa = Transform.zero();
  final Transform _xfb = Transform.zero();

  // TODO_ERIN might not need to return the separation

  double initialize(
    final SimplexCache cache,
    final DistanceProxy proxyA,
    final Sweep sweepA,
    final DistanceProxy proxyB,
    final Sweep sweepB,
    double t1,
  ) {
    this.proxyA = proxyA;
    this.proxyB = proxyB;
    final count = cache.count;
    assert(0 < count && count < 3);

    this.sweepA = sweepA;
    this.sweepB = sweepB;

    sweepA.getTransform(_xfa, t1);
    sweepB.getTransform(_xfb, t1);

    if (count == 1) {
      type = SeparationFunctionType.points;
      _localPointA.setFrom(proxyA.getVertex(cache.indexA[0]));
      _localPointB.setFrom(proxyB.getVertex(cache.indexB[0]));
      _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));
      _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));
      axis
        ..setFrom(_pointB)
        ..sub(_pointA);
      return axis.normalize();
    } else if (cache.indexA[0] == cache.indexA[1]) {
      // Two points on B and one on A.
      type = SeparationFunctionType.faceB;

      _localPointB1.setFrom(proxyB.getVertex(cache.indexB[0]));
      _localPointB2.setFrom(proxyB.getVertex(cache.indexB[1]));

      _temp
        ..setFrom(_localPointB2)
        ..sub(_localPointB1);
      _temp.scaleOrthogonalInto(-1.0, axis);
      axis.normalize();

      _normal.setFrom(Rot.mulVec2(_xfb.q, axis));

      localPoint
        ..setFrom(_localPointB1)
        ..add(_localPointB2)
        ..scale(.5);
      _pointB.setFrom(Transform.mulVec2(_xfb, localPoint));

      _localPointA.setFrom(proxyA.getVertex(cache.indexA[0]));
      _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));

      _temp
        ..setFrom(_pointA)
        ..sub(_pointB);
      var s = _temp.dot(_normal);
      if (s < 0.0) {
        axis.negate();
        s = -s;
      }
      return s;
    } else {
      // Two points on A and one or two points on B.
      type = SeparationFunctionType.faceA;

      _localPointA1.setFrom(proxyA.getVertex(cache.indexA[0]));
      _localPointA2.setFrom(proxyA.getVertex(cache.indexA[1]));

      _temp
        ..setFrom(_localPointA2)
        ..sub(_localPointA1);
      _temp.scaleOrthogonalInto(-1.0, axis);
      axis.normalize();

      _normal.setFrom(Rot.mulVec2(_xfa.q, axis));

      localPoint
        ..setFrom(_localPointA1)
        ..add(_localPointA2)
        ..scale(.5);
      _pointA.setFrom(Transform.mulVec2(_xfa, localPoint));

      _localPointB.setFrom(proxyB.getVertex(cache.indexB[0]));
      _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));

      _temp
        ..setFrom(_pointB)
        ..sub(_pointA);
      var s = _temp.dot(_normal);
      if (s < 0.0) {
        axis.negate();
        s = -s;
      }
      return s;
    }
  }

  final Vector2 _axisA = Vector2.zero();
  final Vector2 _axisB = Vector2.zero();

  // double FindMinSeparation(int* indexA, int* indexB, double t) const
  double findMinSeparation(List<int> indexes, double t) {
    sweepA.getTransform(_xfa, t);
    sweepB.getTransform(_xfb, t);

    switch (type) {
      case SeparationFunctionType.points:
        _axisA.setFrom(Rot.mulTransVec2(_xfa.q, axis));
        _axisB.setFrom(Rot.mulTransVec2(_xfb.q, axis..negate()));
        axis.negate();

        indexes[0] = proxyA.getSupport(_axisA);
        indexes[1] = proxyB.getSupport(_axisB);

        _localPointA.setFrom(proxyA.getVertex(indexes[0]));
        _localPointB.setFrom(proxyB.getVertex(indexes[1]));

        _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));
        _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));

        return (_pointB..sub(_pointA)).dot(axis);
      case SeparationFunctionType.faceA:
        _normal.setFrom(Rot.mulVec2(_xfa.q, axis));
        _pointA.setFrom(Transform.mulVec2(_xfa, localPoint));

        _axisB.setFrom(Rot.mulTransVec2(_xfb.q, _normal..negate()));
        _normal.negate();

        indexes[0] = -1;
        indexes[1] = proxyB.getSupport(_axisB);

        _localPointB.setFrom(proxyB.getVertex(indexes[1]));
        _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));

        return (_pointB..sub(_pointA)).dot(_normal);
      case SeparationFunctionType.faceB:
        _normal.setFrom(Rot.mulVec2(_xfb.q, axis));
        _pointB.setFrom(Transform.mulVec2(_xfb, localPoint));

        _axisA.setFrom(Rot.mulTransVec2(_xfa.q, _normal..negate()));
        _normal.negate();

        indexes[1] = -1;
        indexes[0] = proxyA.getSupport(_axisA);

        _localPointA.setFrom(proxyA.getVertex(indexes[0]));
        _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));

        return (_pointA..sub(_pointB)).dot(_normal);
      default:
        assert(false);
        indexes[0] = -1;
        indexes[1] = -1;
        return 0.0;
    }
  }

  double evaluate(int indexA, int indexB, double t) {
    sweepA.getTransform(_xfa, t);
    sweepB.getTransform(_xfb, t);

    switch (type) {
      case SeparationFunctionType.points:
        _localPointA.setFrom(proxyA.getVertex(indexA));
        _localPointB.setFrom(proxyB.getVertex(indexB));

        _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));
        _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));

        return (_pointB..sub(_pointA)).dot(axis);
      case SeparationFunctionType.faceA:
        _normal.setFrom(Rot.mulVec2(_xfa.q, axis));
        _pointA.setFrom(Transform.mulVec2(_xfa, localPoint));

        _localPointB.setFrom(proxyB.getVertex(indexB));
        _pointB.setFrom(Transform.mulVec2(_xfb, _localPointB));
        return (_pointB..sub(_pointA)).dot(_normal);
      case SeparationFunctionType.faceB:
        _normal.setFrom(Rot.mulVec2(_xfb.q, axis));
        _pointB.setFrom(Transform.mulVec2(_xfb, localPoint));

        _localPointA.setFrom(proxyA.getVertex(indexA));
        _pointA.setFrom(Transform.mulVec2(_xfa, _localPointA));

        return (_pointA..sub(_pointB)).dot(_normal);
      default:
        assert(false);
        return 0.0;
    }
  }
}
