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

/// A particle group definition holds all the data needed to construct a particle group. You can
/// safely re-use these definitions.
class ParticleGroupDef {
  /// The particle-behavior flags.
  int flags = 0;

  /// The group-construction flags.
  int groupFlags = 0;

  /// The world position of the group. Moves the group's shape a distance equal to the value of
  /// position.
  final Vector2 position = new Vector2.zero();

  /// The world angle of the group in radians. Rotates the shape by an angle equal to the value of
  /// angle.
  double angle = 0.0;

  /// The linear velocity of the group's origin in world co-ordinates.
  final Vector2 linearVelocity = new Vector2.zero();

  /// The angular velocity of the group.
  double angularVelocity = 0.0;

  /// The color of all particles in the group.
  ParticleColor color;

  /// The strength of cohesion among the particles in a group with flag b2_elasticParticle or
  /// b2_springParticle.
  double strength = 1.0;

  /// Shape containing the particle group.
  Shape shape;

  /// If true, destroy the group automatically after its last particle has been destroyed.
  bool destroyAutomatically = true;

  /// Use this to store application-specific group data.
  Object userData;
}
