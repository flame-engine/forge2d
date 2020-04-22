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

/// Wheel joint definition. This requires defining a line of motion using an axis and an anchor
/// point. The definition uses local anchor points and a local axis so that the initial configuration
/// can violate the constraint slightly. The joint translation is zero when the local anchor points
/// coincide in world space. Using local anchors and a local axis helps when saving and loading a
/// game.
class WheelJointDef extends JointDef {
  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = Vector2.zero();

  /// The local translation axis in body1.
  final Vector2 localAxisA = Vector2.zero();

  /// Enable/disable the joint motor.
  bool enableMotor = false;

  /// The maximum motor torque, usually in N-m.
  double maxMotorTorque = 0.0;

  /// The desired motor speed in radians per second.
  double motorSpeed = 0.0;

  /// Suspension frequency, zero indicates no suspension
  double frequencyHz = 0.0;

  /// Suspension damping ratio, one indicates critical damping
  double dampingRatio = 0.0;

  WheelJointDef() : super(JointType.WHEEL) {
    localAxisA.setValues(1.0, 0.0);
  }

  void initialize(Body b1, Body b2, Vector2 anchor, Vector2 axis) {
    bodyA = b1;
    bodyB = b2;
    b1.getLocalPointToOut(anchor, localAnchorA);
    b2.getLocalPointToOut(anchor, localAnchorB);
    bodyA.getLocalVectorToOut(axis, localAxisA);
  }
}
