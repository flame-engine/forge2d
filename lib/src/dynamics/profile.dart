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
