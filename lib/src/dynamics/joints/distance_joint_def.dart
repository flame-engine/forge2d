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

/// Distance joint definition. This requires defining an anchor point on both bodies and the non-zero
/// length of the distance joint. The definition uses local anchor points so that the initial
/// configuration can violate the constraint slightly. This helps when saving and loading a game.
///
/// @warning Do not use a zero or short length.

class DistanceJointDef extends JointDef {
  /// The local anchor point relative to body1's origin.
  final Vector2 localAnchorA = new Vector2.zero();

  /// The local anchor point relative to body2's origin.
  final Vector2 localAnchorB = new Vector2.zero();

  /// The equilibrium length between the anchor points.
  double length = 1.0;

  /// The mass-spring-damper frequency in Hertz.
  double frequencyHz = 0.0;

  /// The damping ratio. 0 = no damping, 1 = critical damping.
  double dampingRatio = 0.0;

  DistanceJointDef() : super(JointType.DISTANCE);

  /// Initialize the bodies, anchors, and length using the world anchors.
  ///
  /// @param b1 First body
  /// @param b2 Second body
  /// @param anchor1 World anchor on first body
  /// @param anchor2 World anchor on second body
  void initialize(final Body b1, final Body b2, final Vector2 anchor1,
      final Vector2 anchor2) {
    bodyA = b1;
    bodyB = b2;
    localAnchorA.setFrom(bodyA.getLocalPoint(anchor1));
    localAnchorB.setFrom(bodyB.getLocalPoint(anchor2));
    Vector2 d = anchor2 - anchor1;
    length = d.length;
  }
}
