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

/// Pulley joint definition. This requires two ground anchors, two dynamic body anchor points, and a
/// pulley ratio.
class PulleyJointDef extends JointDef {
  /// The first ground anchor in world coordinates. This point never moves.
  Vector2 groundAnchorA = new Vector2(-1.0, 1.0);

  /// The second ground anchor in world coordinates. This point never moves.
  Vector2 groundAnchorB = new Vector2(1.0, 1.0);

  /// The local anchor point relative to bodyA's origin.
  Vector2 localAnchorA = new Vector2(-1.0, 0.0);

  /// The local anchor point relative to bodyB's origin.
  Vector2 localAnchorB = new Vector2(1.0, 0.0);

  /// The a reference length for the segment attached to bodyA.
  double lengthA = 0.0;

  /// The a reference length for the segment attached to bodyB.
  double lengthB = 0.0;

  /// The pulley ratio, used to simulate a block-and-tackle.
  double ratio = 1.0;

  PulleyJointDef() : super(JointType.PULLEY) {
    collideConnected = true;
  }

  /// Initialize the bodies, anchors, lengths, max lengths, and ratio using the world anchors.
  void initialize(Body b1, Body b2, Vector2 ga1, Vector2 ga2, Vector2 anchor1,
      Vector2 anchor2, double r) {
    bodyA = b1;
    bodyB = b2;
    groundAnchorA = ga1;
    groundAnchorB = ga2;
    localAnchorA = bodyA.getLocalPoint(anchor1);
    localAnchorB = bodyB.getLocalPoint(anchor2);
    Vector2 d1 = anchor1 - ga1;
    lengthA = d1.length;
    Vector2 d2 = anchor2 - ga2;
    lengthB = d2.length;
    ratio = r;
    assert(ratio > Settings.EPSILON);
  }
}
