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

/// The particle type. Can be combined with | operator. Zero means liquid.
class ParticleType {
  static const int b2_waterParticle = 0;

  /// removed after next step
  static const int b2_zombieParticle = 1 << 1;

  /// zero velocity
  static const int b2_wallParticle = 1 << 2;

  /// with restitution from stretching
  static const int b2_springParticle = 1 << 3;

  /// with restitution from deformation
  static const int b2_elasticParticle = 1 << 4;

  /// with viscosity
  static const int b2_viscousParticle = 1 << 5;

  /// without isotropic pressure
  static const int b2_powderParticle = 1 << 6;

  /// with surface tension
  static const int b2_tensileParticle = 1 << 7;

  /// mixing color between contacting particles
  static const int b2_colorMixingParticle = 1 << 8;

  /// call b2DestructionListener on destruction
  static const int b2_destructionListener = 1 << 9;
}
