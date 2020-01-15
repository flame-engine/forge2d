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

class WeldJointDef extends JointDef {
  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = new Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = new Vector2.zero();

  /// The body2 angle minus body1 angle in the reference state (radians).
  double referenceAngle = 0.0;

  /// The mass-spring-damper frequency in Hertz. Rotation only. Disable softness with a value of 0.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  WeldJointDef() : super(JointType.WELD);

  /// Initialize the bodies, anchors, and reference angle using a world anchor point.
  ///
  /// @param bA
  /// @param bB
  /// @param anchor
  void initialize(Body bA, Body bB, Vector2 anchor) {
    bodyA = bA;
    bodyB = bB;
    bodyA.getLocalPointToOut(anchor, localAnchorA);
    bodyB.getLocalPointToOut(anchor, localAnchorB);
    referenceAngle = bodyB.getAngle() - bodyA.getAngle();
  }
}
