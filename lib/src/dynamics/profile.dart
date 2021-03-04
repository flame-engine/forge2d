import 'dart:math' as math;

class ProfileEntry {
  static const int _longAvgNums = 20;
  static const double _longFraction = 1.0 / _longAvgNums;
  static const int _shortAvgNums = 5;
  static const double _shortFraction = 1.0 / _shortAvgNums;

  double longAvg = 0.0;
  double shortAvg = 0.0;
  double min = double.maxFinite;
  double max = -double.maxFinite;
  double _accum = 0.0;

  void record(double value) {
    longAvg = longAvg * (1 - _longFraction) + value * _longFraction;
    shortAvg = shortAvg * (1 - _shortFraction) + value * _shortFraction;
    min = math.min(value, min);
    max = math.max(value, max);
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

  @override
  String toString() {
    return '$shortAvg ($longAvg) [$min,$max]';
  }
}

class Profile {
  final ProfileEntry step = ProfileEntry();
  final ProfileEntry stepInit = ProfileEntry();
  final ProfileEntry collide = ProfileEntry();
  final ProfileEntry solveParticleSystem = ProfileEntry();
  final ProfileEntry solve = ProfileEntry();
  final ProfileEntry solveInit = ProfileEntry();
  final ProfileEntry solveVelocity = ProfileEntry();
  final ProfileEntry solvePosition = ProfileEntry();
  final ProfileEntry broadphase = ProfileEntry();
  final ProfileEntry solveTOI = ProfileEntry();

  void toDebugStrings(List<String> strings) {
    strings.add('Profile:');
    strings.add(' step: $step');
    strings.add('  init: $stepInit');
    strings.add('  collide: $collide');
    strings.add('  particles: $solveParticleSystem');
    strings.add('  solve: $solve');
    strings.add('   solveInit: $solveInit');
    strings.add('   solveVelocity: $solveVelocity');
    strings.add('   solvePosition: $solvePosition');
    strings.add('   broadphase: $broadphase');
    strings.add('  solveTOI: $solveTOI');
  }
}
