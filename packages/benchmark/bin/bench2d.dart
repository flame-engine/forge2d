// The classic bench2d benchmark: a 40-high pyramid of boxes on flat ground,
// 256 warm-up frames followed by 256 measured frames.
// Run with: dart run bin/bench2d.dart
// ignore_for_file: avoid_print
import 'package:forge2d/forge2d.dart';

const frames = 256;
const pyramidSize = 40;
const timeStep = 1.0 / 60.0;

Future<void> main() async {
  await initializeForge2D();

  final bench = Bench2d()..initialize();
  bench.warmUp();
  final times = bench.bench();

  final mean = times.reduce((a, b) => a + b) / times.length;
  double percentile(double rank) => times[(rank * times.length) ~/ 100];
  print('Benchmark complete.');
  print('ms/frame: ${mean.toStringAsFixed(3)}');
  print('5th percentile: ${percentile(5)}');
  print('95th percentile: ${percentile(95)}');
}

class Bench2d {
  final World world = World();

  void initialize() {
    world.createBody().createShape(
      Segment(point1: Vector2(-40, -30), point2: Vector2(40, -30)),
    );

    // The pyramid: 40 rows of half-extent 0.5 boxes.
    const boxSize = 0.5;
    final x = Vector2(-7, 0.75);
    final y = Vector2.zero();
    final deltaX = Vector2(0.5625, 1);
    final deltaY = Vector2(1.125, 0);

    for (var i = 0; i < pyramidSize; ++i) {
      y.setFrom(x);
      for (var j = i; j < pyramidSize; ++j) {
        world
            .createBody(BodyDef(type: BodyType.dynamic, position: y))
            .createShape(Polygon.square(boxSize), ShapeDef(density: 5));
        y.add(deltaY);
      }
      x.add(deltaX);
    }
  }

  void warmUp() {
    for (var i = 0; i < frames; ++i) {
      world.step(timeStep);
    }
  }

  /// Returns the sorted per-frame times in milliseconds.
  List<double> bench() {
    final stopwatch = Stopwatch()..start();
    final times = List<double>.generate(frames, (_) {
      final begin = stopwatch.elapsedMicroseconds;
      world.step(timeStep);
      return (stopwatch.elapsedMicroseconds - begin) / 1000;
    });
    return times..sort();
  }
}
