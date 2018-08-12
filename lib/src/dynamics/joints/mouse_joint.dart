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
 * A mouse joint is used to make a point on a body track a specified world point. This a soft
 * constraint with a maximum force. This allows the constraint to stretch and without applying huge
 * forces. NOTE: this joint is not documented in the manual because it was developed to be used in
 * the testbed. If you want to learn how to use the mouse joint, look at the testbed.
 *
 * @author Daniel
 */
class MouseJoint extends Joint {
  final Vector2 _localAnchorB = new Vector2.zero();
  final Vector2 _targetA = new Vector2.zero();
  double _frequencyHz = 0.0;
  double _dampingRatio = 0.0;
  double _beta = 0.0;

  // Solver shared
  final Vector2 _impulse = new Vector2.zero();
  double _maxForce = 0.0;
  double _gamma = 0.0;

  // Solver temp
  int _indexB = 0;
  final Vector2 _rB = new Vector2.zero();
  final Vector2 _localCenterB = new Vector2.zero();
  double _invMassB = 0.0;
  double _invIB = 0.0;
  final Matrix2 _mass = new Matrix2.zero();
  final Vector2 _C = new Vector2.zero();

  MouseJoint(IWorldPool argWorld, MouseJointDef def) : super(argWorld, def) {
    assert(MathUtils.vector2IsValid(def.target));
    assert(def.maxForce >= 0);
    assert(def.frequencyHz >= 0);
    assert(def.dampingRatio >= 0);

    _targetA.setFrom(def.target);
    Transform.mulTransToOutUnsafeVec2(
        _bodyB._transform, _targetA, _localAnchorB);

    _maxForce = def.maxForce;
    _impulse.setZero();

    _frequencyHz = def.frequencyHz;
    _dampingRatio = def.dampingRatio;
  }

  void getAnchorA(Vector2 argOut) {
    argOut.setFrom(_targetA);
  }

  void getAnchorB(Vector2 argOut) {
    _bodyB.getWorldPointToOut(_localAnchorB, argOut);
  }

  void getReactionForce(double invDt, Vector2 argOut) {
    argOut
      ..setFrom(_impulse)
      ..scale(invDt);
  }

  double getReactionTorque(double invDt) {
    return invDt * 0.0;
  }

  void setTarget(Vector2 target) {
    if (_bodyB.isAwake() == false) {
      _bodyB.setAwake(true);
    }
    _targetA.setFrom(target);
  }

  Vector2 getTarget() {
    return _targetA;
  }

  void initVelocityConstraints(final SolverData data) {
    _indexB = _bodyB._islandIndex;
    _localCenterB.setFrom(_bodyB._sweep.localCenter);
    _invMassB = _bodyB._invMass;
    _invIB = _bodyB._invI;

    Vector2 cB = data.positions[_indexB].c;
    double aB = data.positions[_indexB].a;
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    final Rot qB = pool.popRot();

    qB.setAngle(aB);

    double mass = _bodyB.mass;

    // Frequency
    double omega = 2.0 * Math.pi * _frequencyHz;

    // Damping coefficient
    double d = 2.0 * mass * _dampingRatio * omega;

    // Spring stiffness
    double k = mass * (omega * omega);

    // magic formulas
    // gamma has units of inverse mass.
    // beta has units of inverse time.
    double h = data.step.dt;
    assert(d + h * k > Settings.EPSILON);
    _gamma = h * (d + h * k);
    if (_gamma != 0.0) {
      _gamma = 1.0 / _gamma;
    }
    _beta = h * k * _gamma;

    Vector2 temp = pool.popVec2();

    // Compute the effective mass matrix.
    Rot.mulToOutUnsafe(
        qB,
        temp
          ..setFrom(_localAnchorB)
          ..sub(_localCenterB),
        _rB);

    // K = [(1/m1 + 1/m2) * eye(2) - skew(r1) * invI1 * skew(r1) - skew(r2) * invI2 * skew(r2)]
    // = [1/m1+1/m2 0 ] + invI1 * [r1.y*r1.y -r1.x*r1.y] + invI2 * [r1.y*r1.y -r1.x*r1.y]
    // [ 0 1/m1+1/m2] [-r1.x*r1.y r1.x*r1.x] [-r1.x*r1.y r1.x*r1.x]
    final Matrix2 K = pool.popMat22();
    double a11 = _invMassB + _invIB * _rB.y * _rB.y + _gamma;
    double a21 = -_invIB * _rB.x * _rB.y;
    double a12 = a21;
    double a22 = _invMassB + _invIB * _rB.x * _rB.x + _gamma;

    K.setValues(a11, a21, a12, a22);
    _mass.setFrom(K);
    _mass.invert();

    _C
      ..setFrom(cB)
      ..add(_rB)
      ..sub(_targetA);
    _C.scale(_beta);

    // Cheat with some damping
    wB *= 0.98;

    if (data.step.warmStarting) {
      _impulse.scale(data.step.dtRatio);
      vB.x += _invMassB * _impulse.x;
      vB.y += _invMassB * _impulse.y;
      wB += _invIB * _rB.cross(_impulse);
    } else {
      _impulse.setZero();
    }

//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushVec2(1);
    pool.pushMat22(1);
    pool.pushRot(1);
  }

  bool solvePositionConstraints(final SolverData data) {
    return true;
  }

  void solveVelocityConstraints(final SolverData data) {
    Vector2 vB = data.velocities[_indexB].v;
    double wB = data.velocities[_indexB].w;

    // Cdot = v + cross(w, r)
    final Vector2 Cdot = pool.popVec2();
    _rB.scaleOrthogonalInto(wB, Cdot);
    Cdot.add(vB);

    final Vector2 impulse = pool.popVec2();
    final Vector2 temp = pool.popVec2();

    temp
      ..setFrom(_impulse)
      ..scale(_gamma)
      ..add(_C)
      ..add(Cdot)
      ..negate();
    _mass.transformed(temp, impulse);

    Vector2 oldImpulse = temp;
    oldImpulse.setFrom(_impulse);
    _impulse.add(impulse);
    double maxImpulse = data.step.dt * _maxForce;
    if (_impulse.length2 > maxImpulse * maxImpulse) {
      _impulse.scale(maxImpulse / _impulse.length);
    }
    impulse
      ..setFrom(_impulse)
      ..sub(oldImpulse);

    vB.x += _invMassB * impulse.x;
    vB.y += _invMassB * impulse.y;
    wB += _invIB * _rB.cross(impulse);

//    data.velocities[_indexB].v.set(vB);
    data.velocities[_indexB].w = wB;

    pool.pushVec2(3);
  }
}
