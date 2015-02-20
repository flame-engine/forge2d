/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

part of box2d;

/**
 * Input parameters for TOI
 * 
 * @author Daniel Murphy
 */
class TOIInput {
  final DistanceProxy proxyA = new DistanceProxy();
  final DistanceProxy proxyB = new DistanceProxy();
  final Sweep sweepA = new Sweep();
  final Sweep sweepB = new Sweep();
  /**
   * defines sweep interval [0, tMax]
   */
  double tMax = 0.0;
}

enum TOIOutputState { UNKNOWN, FAILED, OVERLAPPED, TOUCHING, SEPARATED }

/**
 * Output parameters for TimeOfImpact
 * 
 * @author daniel
 */
class TOIOutput {
  TOIOutputState state = TOIOutputState.UNKNOWN;
  double t = 0.0;
}

/**
 * Class used for computing the time of impact. This class should not be constructed usually, just
 * retrieve from the {@link SingletonPool#getTOI()}.
 * 
 * @author daniel
 */
class TimeOfImpact {
  static const int MAX_ITERATIONS = 1000;

  static int toiCalls = 0;
  static int toiIters = 0;
  static int toiMaxIters = 0;
  static int toiRootIters = 0;
  static int toiMaxRootIters = 0;

  // djm pooling
  final SimplexCache _cache = new SimplexCache();
  final DistanceInput _distanceInput = new DistanceInput();
  final Transform _xfA = new Transform.zero();
  final Transform _xfB = new Transform.zero();
  final DistanceOutput _distanceOutput = new DistanceOutput();
  final SeparationFunction _fcn = new SeparationFunction();
  final List<int> _indexes = BufferUtils.allocClearIntList(2);
  final Sweep _sweepA = new Sweep();
  final Sweep _sweepB = new Sweep();

  final IWorldPool _pool;

  TimeOfImpact(this._pool);

  /**
   * Compute the upper bound on time before two shapes penetrate. Time is represented as a fraction
   * between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
   * non-tunneling collision. If you change the time interval, you should call this function again.
   * Note: use Distance to compute the contact point and normal at the time of impact.
   * 
   * @param output
   * @param input
   */
  void timeOfImpact(TOIOutput output, TOIInput input) {
    // CCD via the local separating axis method. This seeks progression
    // by computing the largest time at which separation is maintained.

    ++toiCalls;

    output.state = TOIOutputState.UNKNOWN;
    output.t = input.tMax;

    final DistanceProxy proxyA = input.proxyA;
    final DistanceProxy proxyB = input.proxyB;

    _sweepA.set(input.sweepA);
    _sweepB.set(input.sweepB);

    // Large rotations can make the root finder fail, so we normalize the
    // sweep angles.
    _sweepA.normalize();
    _sweepB.normalize();

    double tMax = input.tMax;

    double totalRadius = proxyA.m_radius + proxyB.m_radius;
    // djm: whats with all these constants?
    double target =
        Math.max(Settings.linearSlop, totalRadius - 3.0 * Settings.linearSlop);
    double tolerance = 0.25 * Settings.linearSlop;

    assert(target > tolerance);

    double t1 = 0.0;
    int iter = 0;

    _cache.count = 0;
    _distanceInput.proxyA = input.proxyA;
    _distanceInput.proxyB = input.proxyB;
    _distanceInput.useRadii = false;

    // The outer loop progressively attempts to compute new separating axes.
    // This loop terminates when an axis is repeated (no progress is made).
    for (;;) {
      _sweepA.getTransform(_xfA, t1);
      _sweepB.getTransform(_xfB, t1);
      // System.out.printf("sweepA: %f, %f, sweepB: %f, %f\n",
      // sweepA.c.x, sweepA.c.y, sweepB.c.x, sweepB.c.y);
      // Get the distance between shapes. We can also use the results
      // to get a separating axis
      _distanceInput.transformA = _xfA;
      _distanceInput.transformB = _xfB;
      _pool.getDistance().distance(_distanceOutput, _cache, _distanceInput);

      // System.out.printf("Dist: %f at points %f, %f and %f, %f.  %d iterations\n",
      // distanceOutput.distance, distanceOutput.pointA.x, distanceOutput.pointA.y,
      // distanceOutput.pointB.x, distanceOutput.pointB.y,
      // distanceOutput.iterations);

      // If the shapes are overlapped, we give up on continuous collision.
      if (_distanceOutput.distance <= 0.0) {
        // Failure!
        output.state = TOIOutputState.OVERLAPPED;
        output.t = 0.0;
        break;
      }

      if (_distanceOutput.distance < target + tolerance) {
        // Victory!
        output.state = TOIOutputState.TOUCHING;
        output.t = t1;
        break;
      }

      // Initialize the separating axis.
      _fcn.initialize(_cache, proxyA, _sweepA, proxyB, _sweepB, t1);

      // Compute the TOI on the separating axis. We do this by successively
      // resolving the deepest point. This loop is bounded by the number of
      // vertices.
      bool done = false;
      double t2 = tMax;
      int pushBackIter = 0;
      for (;;) {

        // Find the deepest point at t2. Store the witness point indices.
        double s2 = _fcn.findMinSeparation(_indexes, t2);
        // System.out.printf("s2: %f\n", s2);
        // Is the final configuration separated?
        if (s2 > target + tolerance) {
          // Victory!
          output.state = TOIOutputState.SEPARATED;
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
        double s1 = _fcn.evaluate(_indexes[0], _indexes[1], t1);
        // Check for initial overlap. This might happen if the root finder
        // runs out of iterations.
        // System.out.printf("s1: %f, target: %f, tolerance: %f\n", s1, target,
        // tolerance);
        if (s1 < target - tolerance) {
          output.state = TOIOutputState.FAILED;
          output.t = t1;
          done = true;
          break;
        }

        // Check for touching
        if (s1 <= target + tolerance) {
          // Victory! t1 should hold the TOI (could be 0.0).
          output.state = TOIOutputState.TOUCHING;
          output.t = t1;
          done = true;
          break;
        }

        // Compute 1D root of: f(x) - target = 0
        int rootIterCount = 0;
        double a1 = t1,
            a2 = t2;
        for (;;) {
          // Use a mix of the secant rule and bisection.
          double t;
          if ((rootIterCount & 1) == 1) {
            // Secant rule to improve convergence.
            t = a1 + (target - s1) * (a2 - a1) / (s2 - s1);
          } else {
            // Bisection to guarantee progress.
            t = 0.5 * (a1 + a2);
          }

          double s = _fcn.evaluate(_indexes[0], _indexes[1], t);

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

          ++rootIterCount;
          ++toiRootIters;

          // djm: whats with this? put in settings?
          if (rootIterCount == 50) {
            break;
          }
        }

        toiMaxRootIters = Math.max(toiMaxRootIters, rootIterCount);

        ++pushBackIter;

        if (pushBackIter == Settings.maxPolygonVertices) {
          break;
        }
      }

      ++iter;
      ++toiIters;

      if (done) {
        // System.out.println("done");
        break;
      }

      if (iter == MAX_ITERATIONS) {
        // System.out.println("failed, root finder stuck");
        // Root finder got stuck. Semi-victory.
        output.state = TOIOutputState.FAILED;
        output.t = t1;
        break;
      }
    }

    // System.out.printf("final sweeps: %f, %f, %f; %f, %f, %f", input.s)
    toiMaxIters = Math.max(toiMaxIters, iter);
  }
} // Class TimeOfImpact.

enum SeparationFunctionType { POINTS, FACE_A, FACE_B }

class SeparationFunction {
  DistanceProxy m_proxyA;
  DistanceProxy m_proxyB;
  SeparationFunctionType m_type;
  final Vec2 m_localPoint = new Vec2.zero();
  final Vec2 m_axis = new Vec2.zero();
  Sweep m_sweepA;
  Sweep m_sweepB;

  // djm pooling
  final Vec2 _localPointA = new Vec2.zero();
  final Vec2 _localPointB = new Vec2.zero();
  final Vec2 _pointA = new Vec2.zero();
  final Vec2 _pointB = new Vec2.zero();
  final Vec2 _localPointA1 = new Vec2.zero();
  final Vec2 _localPointA2 = new Vec2.zero();
  final Vec2 _normal = new Vec2.zero();
  final Vec2 _localPointB1 = new Vec2.zero();
  final Vec2 _localPointB2 = new Vec2.zero();
  final Vec2 _temp = new Vec2.zero();
  final Transform _xfa = new Transform.zero();
  final Transform _xfb = new Transform.zero();

  // TODO_ERIN might not need to return the separation

  double initialize(final SimplexCache cache, final DistanceProxy proxyA,
      final Sweep sweepA, final DistanceProxy proxyB, final Sweep sweepB,
      double t1) {
    m_proxyA = proxyA;
    m_proxyB = proxyB;
    int count = cache.count;
    assert(0 < count && count < 3);

    m_sweepA = sweepA;
    m_sweepB = sweepB;

    m_sweepA.getTransform(_xfa, t1);
    m_sweepB.getTransform(_xfb, t1);

    // log.debug("initializing separation.\n" +
    // "cache: "+cache.count+"-"+cache.metric+"-"+cache.indexA+"-"+cache.indexB+"\n"
    // "distance: "+proxyA.

    if (count == 1) {
      m_type = SeparationFunctionType.POINTS;
      /*
       * Vec2 localPointA = m_proxyA.GetVertex(cache.indexA[0]); Vec2 localPointB =
       * m_proxyB.GetVertex(cache.indexB[0]); Vec2 pointA = Mul(transformA, localPointA); Vec2
       * pointB = Mul(transformB, localPointB); m_axis = pointB - pointA; m_axis.Normalize();
       */
      _localPointA.set(m_proxyA.getVertex(cache.indexA[0]));
      _localPointB.set(m_proxyB.getVertex(cache.indexB[0]));
      Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);
      Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);
      m_axis.set(_pointB).subLocal(_pointA);
      double s = m_axis.normalize();
      return s;
    } else if (cache.indexA[0] == cache.indexA[1]) {
      // Two points on B and one on A.
      m_type = SeparationFunctionType.FACE_B;

      _localPointB1.set(m_proxyB.getVertex(cache.indexB[0]));
      _localPointB2.set(m_proxyB.getVertex(cache.indexB[1]));

      _temp.set(_localPointB2).subLocal(_localPointB1);
      Vec2.crossToOutUnsafeVec2Dbl(_temp, 1.0, m_axis);
      m_axis.normalize();

      Rot.mulToOutUnsafe(_xfb.q, m_axis, _normal);

      m_localPoint.set(_localPointB1).addLocal(_localPointB2).mulLocal(.5);
      Transform.mulToOutUnsafeVec2(_xfb, m_localPoint, _pointB);

      _localPointA.set(proxyA.getVertex(cache.indexA[0]));
      Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);

      _temp.set(_pointA).subLocal(_pointB);
      double s = Vec2.dot(_temp, _normal);
      if (s < 0.0) {
        m_axis.negateLocal();
        s = -s;
      }
      return s;
    } else {
      // Two points on A and one or two points on B.
      m_type = SeparationFunctionType.FACE_A;

      _localPointA1.set(m_proxyA.getVertex(cache.indexA[0]));
      _localPointA2.set(m_proxyA.getVertex(cache.indexA[1]));

      _temp.set(_localPointA2).subLocal(_localPointA1);
      Vec2.crossToOutUnsafeVec2Dbl(_temp, 1.0, m_axis);
      m_axis.normalize();

      Rot.mulToOutUnsafe(_xfa.q, m_axis, _normal);

      m_localPoint.set(_localPointA1).addLocal(_localPointA2).mulLocal(.5);
      Transform.mulToOutUnsafeVec2(_xfa, m_localPoint, _pointA);

      _localPointB.set(m_proxyB.getVertex(cache.indexB[0]));
      Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);

      _temp.set(_pointB).subLocal(_pointA);
      double s = Vec2.dot(_temp, _normal);
      if (s < 0.0) {
        m_axis.negateLocal();
        s = -s;
      }
      return s;
    }
  }

  final Vec2 _axisA = new Vec2.zero();
  final Vec2 _axisB = new Vec2.zero();

  // double FindMinSeparation(int* indexA, int* indexB, double t) const
  double findMinSeparation(List<int> indexes, double t) {
    m_sweepA.getTransform(_xfa, t);
    m_sweepB.getTransform(_xfb, t);

    switch (m_type) {
      case SeparationFunctionType.POINTS:
        Rot.mulTransUnsafeVec2(_xfa.q, m_axis, _axisA);
        Rot.mulTransUnsafeVec2(_xfb.q, m_axis.negateLocal(), _axisB);
        m_axis.negateLocal();

        indexes[0] = m_proxyA.getSupport(_axisA);
        indexes[1] = m_proxyB.getSupport(_axisB);

        _localPointA.set(m_proxyA.getVertex(indexes[0]));
        _localPointB.set(m_proxyB.getVertex(indexes[1]));

        Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);
        Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);

        double separation = Vec2.dot(_pointB.subLocal(_pointA), m_axis);
        return separation;

      case SeparationFunctionType.FACE_A:
        Rot.mulToOutUnsafe(_xfa.q, m_axis, _normal);
        Transform.mulToOutUnsafeVec2(_xfa, m_localPoint, _pointA);

        Rot.mulTransUnsafeVec2(_xfb.q, _normal.negateLocal(), _axisB);
        _normal.negateLocal();

        indexes[0] = -1;
        indexes[1] = m_proxyB.getSupport(_axisB);

        _localPointB.set(m_proxyB.getVertex(indexes[1]));
        Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);

        double separation = Vec2.dot(_pointB.subLocal(_pointA), _normal);
        return separation;

      case SeparationFunctionType.FACE_B:
        Rot.mulToOutUnsafe(_xfb.q, m_axis, _normal);
        Transform.mulToOutUnsafeVec2(_xfb, m_localPoint, _pointB);

        Rot.mulTransUnsafeVec2(_xfa.q, _normal.negateLocal(), _axisA);
        _normal.negateLocal();

        indexes[1] = -1;
        indexes[0] = m_proxyA.getSupport(_axisA);

        _localPointA.set(m_proxyA.getVertex(indexes[0]));
        Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);

        double separation = Vec2.dot(_pointA.subLocal(_pointB), _normal);
        return separation;

      default:
        assert(false);
        indexes[0] = -1;
        indexes[1] = -1;
        return 0.0;
    }
  }

  double evaluate(int indexA, int indexB, double t) {
    m_sweepA.getTransform(_xfa, t);
    m_sweepB.getTransform(_xfb, t);

    switch (m_type) {
      case SeparationFunctionType.POINTS:
        Rot.mulTransUnsafeVec2(_xfa.q, m_axis, _axisA);
        Rot.mulTransUnsafeVec2(_xfb.q, m_axis.negateLocal(), _axisB);
        m_axis.negateLocal();

        _localPointA.set(m_proxyA.getVertex(indexA));
        _localPointB.set(m_proxyB.getVertex(indexB));

        Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);
        Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);

        double separation = Vec2.dot(_pointB.subLocal(_pointA), m_axis);
        return separation;

      case SeparationFunctionType.FACE_A:
        Rot.mulToOutUnsafe(_xfa.q, m_axis, _normal);
        Transform.mulToOutUnsafeVec2(_xfa, m_localPoint, _pointA);

        Rot.mulTransUnsafeVec2(_xfb.q, _normal.negateLocal(), _axisB);
        _normal.negateLocal();

        _localPointB.set(m_proxyB.getVertex(indexB));
        Transform.mulToOutUnsafeVec2(_xfb, _localPointB, _pointB);
        double separation = Vec2.dot(_pointB.subLocal(_pointA), _normal);
        return separation;

      case SeparationFunctionType.FACE_B:
        Rot.mulToOutUnsafe(_xfb.q, m_axis, _normal);
        Transform.mulToOutUnsafeVec2(_xfb, m_localPoint, _pointB);

        Rot.mulTransUnsafeVec2(_xfa.q, _normal.negateLocal(), _axisA);
        _normal.negateLocal();

        _localPointA.set(m_proxyA.getVertex(indexA));
        Transform.mulToOutUnsafeVec2(_xfa, _localPointA, _pointA);

        double separation = Vec2.dot(_pointA.subLocal(_pointB), _normal);
        return separation;

      default:
        assert(false);
        return 0.0;
    }
  }
}
