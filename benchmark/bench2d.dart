import '../lib/box2d.dart';

void main() {
  Bench2d()
    ..initialize()
    ..warmup()
    ..bench();
}

void log(String msg) {
  if (DEBUG) {
    print(msg);
  }
}

const bool DEBUG = const bool.fromEnvironment('debug', defaultValue: false);

const bool CHECKSUM =
    const bool.fromEnvironment('checksum', defaultValue: false);

const int FRAMES = 256;
const int PYRAMID_SIZE = 40;

const double _timeStep = 1.0 / 60.0;

double meanF(List<double> values) {
  double total = 0.0;
  for (int i = 0; i < FRAMES; ++i) {
    total += values[i];
  }
  return total / FRAMES;
}

// Simple nearest-rank %ile (on sorted array). We should have enough samples to make this reasonable.
double percentile(List<double> values, double pc) {
  int rank = (pc * values.length) ~/ 100;
  return values[rank];
}

class Bench2d {
  final World world;

  Bench2d() : world = World.withGravity(Vector2(0.0, -10.0));

  void initialize() {
    BodyDef bd = BodyDef();
    Body ground = world.createBody(bd);

    PolygonShape groundShape = PolygonShape()
      ..setAsEdge(Vector2(-20.0, -30.0), Vector2(20.0, -30.0));
    ground.createFixtureFromShape(groundShape, 0.0);
    //EdgeShape groundShape = EdgeShape()
    //  ..set(Vector2(-40.0, -30.0), Vector2(40.0, -30.0));
    //ground.createFixtureFromShape(groundShape, 0.0);

    // add boxes
    const boxSize = .5;
    PolygonShape shape = PolygonShape()..setAsBoxXY(boxSize, boxSize);

    Vector2 x = Vector2(-7.0, 0.75);
    Vector2 y = Vector2.zero();
    Vector2 deltaX = Vector2(0.5625, 1.0);
    Vector2 deltaY = Vector2(1.125, 0.0);

    for (int i = 0; i < PYRAMID_SIZE; ++i) {
      y.setFrom(x);

      for (int j = i; j < PYRAMID_SIZE; ++j) {
        BodyDef bd = BodyDef()
          ..type = BodyType.DYNAMIC
          ..position.setFrom(y);
        world.createBody(bd)..createFixtureFromShape(shape, 5.0);
        y.add(deltaY);
      }

      x.add(deltaX);
    }
    // TODO: Why does some bodies sleep prematurely
    world.setAllowSleep(false);
  }

  List<double> bench() {
    List<double> times = List<double>(FRAMES);
    Stopwatch stopwatch = Stopwatch()..start();
    for (int i = 0; i < FRAMES; ++i) {
      int begin = stopwatch.elapsedMilliseconds;
      step();
      int end = stopwatch.elapsedMilliseconds;
      times[i] = (end - begin).toDouble();
      log("${times[i]}");
    }

    times.sort();
    double mean = meanF(times);
    double fifth = percentile(times, 5.0);
    double ninetyFifth = percentile(times, 95.0);
    print("Benchmark complete.\n"
        "ms/frame: $mean 5th %ile: ${percentile(times, 5.0)} 95th %ile: ${percentile(times, 95.0)}");
    if (CHECKSUM) {
      checksum(world);
    }
    return <double>[mean, fifth, ninetyFifth];
  }

  void warmup() {
    for (int i = 0; i < FRAMES; ++i) {
      step();
    }
  }

  void step() {
    world.stepDt(_timeStep, 3, 3);
  }

  void checksum(World world) {
    Vector2 positionSum = Vector2.zero();
    Vector2 linearVelocitySum = Vector2.zero();
    double angularVelocitySum = 0.0;
    var checksum = (Body b) {
      positionSum = positionSum + b.position;
      linearVelocitySum = linearVelocitySum + b.linearVelocity;
      angularVelocitySum += b.angularVelocity;
    };
    Body firstBody = world.bodyList;
    print(firstBody);
    world.forEachBody(checksum);
    print(
        "pos: $positionSum linVel $linearVelocitySum angVel $angularVelocitySum");
  }
}
