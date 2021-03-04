import 'package:forge2d/forge2d.dart';

void main() {
  Bench2d()
    ..initialize()
    ..warmUp()
    ..bench();
}

void log(String msg) {
  if (debug) {
    print(msg);
  }
}

const bool debug = false;

const bool checksum = false;

const int frames = 256;
const int pyramidSize = 40;

const double _timeStep = 1.0 / 60.0;

double meanF(List<double> values) {
  var total = 0.0;
  for (var i = 0; i < frames; ++i) {
    total += values[i];
  }
  return total / frames;
}

// Simple nearest-rank %ile (on sorted array). We should have enough samples to make this reasonable.
double percentile(List<double> values, double pc) {
  final rank = (pc * values.length) ~/ 100;
  return values[rank];
}

class Bench2d {
  final World world;

  Bench2d() : world = World(Vector2(0.0, -10.0));

  void initialize() {
    final bd = BodyDef();
    final ground = world.createBody(bd);

    final groundShape = EdgeShape()
      ..set(Vector2(-40.0, -30.0), Vector2(40.0, -30.0));
    ground.createFixtureFromShape(groundShape);

    // add boxes
    const boxSize = .5;
    final shape = PolygonShape()..setAsBoxXY(boxSize, boxSize);

    final x = Vector2(-7.0, 0.75);
    final y = Vector2.zero();
    final deltaX = Vector2(0.5625, 1.0);
    final deltaY = Vector2(1.125, 0.0);

    for (var i = 0; i < pyramidSize; ++i) {
      y.setFrom(x);

      for (var j = i; j < pyramidSize; ++j) {
        final bd = BodyDef()
          ..type = BodyType.dynamic
          ..position.setFrom(y);
        world.createBody(bd)..createFixtureFromShape(shape, 5.0);
        y.add(deltaY);
      }

      x.add(deltaX);
    }
  }

  List<double> bench() {
    final stopwatch = Stopwatch()..start();
    final times = List<double>.generate(frames, (_) {
      final begin = stopwatch.elapsedMilliseconds;
      step();
      final end = stopwatch.elapsedMilliseconds;
      final time = (end - begin).toDouble();
      log('$time');
      return time;
    });

    times.sort();
    final mean = meanF(times);
    final fifth = percentile(times, 5.0);
    final ninetyFifth = percentile(times, 95.0);
    print(
      'Benchmark complete.\n'
      'ms/frame: $mean 5th %ile: ${percentile(times, 5.0)} '
      '95th %ile: ${percentile(times, 95.0)}',
    );
    if (checksum) {
      printChecksum(world);
    }
    return <double>[mean, fifth, ninetyFifth];
  }

  void warmUp() {
    for (var i = 0; i < frames; ++i) {
      step();
    }
  }

  void step() {
    world.stepDt(_timeStep, 3, 3);
  }

  void printChecksum(World world) {
    var positionSum = Vector2.zero();
    var linearVelocitySum = Vector2.zero();
    var angularVelocitySum = 0.0;
    void checksum(Body b) {
      positionSum = positionSum + b.position;
      linearVelocitySum = linearVelocitySum + b.linearVelocity;
      angularVelocitySum += b.angularVelocity;
    }

    print(world.bodies.first);
    world.bodies.forEach(checksum);
    print(
      'pos: $positionSum linVel $linearVelocitySum angVel $angularVelocitySum',
    );
  }
}
