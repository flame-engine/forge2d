class Timer {
  final _stopWatch = Stopwatch();

  Timer() {
    _stopWatch.start();
  }

  void reset() {
    _stopWatch.reset();
  }

  double getMilliseconds() {
    return _stopWatch.elapsedMilliseconds.toDouble();
  }
}
