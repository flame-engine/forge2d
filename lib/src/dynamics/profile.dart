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

class ProfileEntry {
  static const int _LONG_AVG_NUMS = 20;
  static const double _LONG_FRACTION = 1.0 / _LONG_AVG_NUMS;
  static const int _SHORT_AVG_NUMS = 5;
  static const double _SHORT_FRACTION = 1.0 / _SHORT_AVG_NUMS;

  double longAvg = 0.0;
  double shortAvg = 0.0;
  double min = double.maxFinite;
  double max = -double.maxFinite;
  double _accum = 0.0;

  void record(double value) {
    longAvg = longAvg * (1 - _LONG_FRACTION) + value * _LONG_FRACTION;
    shortAvg = shortAvg * (1 - _SHORT_FRACTION) + value * _SHORT_FRACTION;
    min = Math.min(value, min);
    max = Math.max(value, max);
  }

  void startAccum() {
    _accum = 0.0;
  }

  void accum(double value) {
    _accum += value;
  }

  void endAccum() {
    record(_accum);
  }

  String toString() {
    return "$shortAvg ($longAvg) [$min,$max]";
  }
}

class Profile {
  final ProfileEntry step = new ProfileEntry();
  final ProfileEntry stepInit = new ProfileEntry();
  final ProfileEntry collide = new ProfileEntry();
  final ProfileEntry solveParticleSystem = new ProfileEntry();
  final ProfileEntry solve = new ProfileEntry();
  final ProfileEntry solveInit = new ProfileEntry();
  final ProfileEntry solveVelocity = new ProfileEntry();
  final ProfileEntry solvePosition = new ProfileEntry();
  final ProfileEntry broadphase = new ProfileEntry();
  final ProfileEntry solveTOI = new ProfileEntry();

  void toDebugStrings(List<String> strings) {
    strings.add("Profile:");
    strings.add(" step: $step");
    strings.add("  init: $stepInit");
    strings.add("  collide: $collide");
    strings.add("  particles: $solveParticleSystem");
    strings.add("  solve: $solve");
    strings.add("   solveInit: $solveInit");
    strings.add("   solveVelocity: $solveVelocity");
    strings.add("   solvePosition: $solvePosition");
    strings.add("   broadphase: $broadphase");
    strings.add("  solveTOI: $solveTOI");
  }
}
