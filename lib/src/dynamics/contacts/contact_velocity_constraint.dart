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

class VelocityConstraintPoint {
  final Vector2 rA = Vector2.zero();
  final Vector2 rB = Vector2.zero();
  double normalImpulse = 0.0;
  double tangentImpulse = 0.0;
  double normalMass = 0.0;
  double tangentMass = 0.0;
  double velocityBias = 0.0;
}

class ContactVelocityConstraint {
  List<VelocityConstraintPoint> points =
      List<VelocityConstraintPoint>(Settings.maxManifoldPoints);
  final Vector2 normal = Vector2.zero();
  final Matrix2 normalMass = Matrix2.zero();
  final Matrix2 K = Matrix2.zero();
  int indexA = 0;
  int indexB = 0;
  double invMassA = 0.0, invMassB = 0.0;
  double invIA = 0.0, invIB = 0.0;
  double friction = 0.0;
  double restitution = 0.0;
  double tangentSpeed = 0.0;
  int pointCount = 0;
  int contactIndex = 0;

  ContactVelocityConstraint() {
    for (int i = 0; i < points.length; i++) {
      points[i] = VelocityConstraintPoint();
    }
  }
}
