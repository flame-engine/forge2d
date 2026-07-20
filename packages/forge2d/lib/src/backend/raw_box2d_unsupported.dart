import 'package:forge2d/src/backend/raw_box2d.dart';

/// Fails: this platform has no forge2d backend.
Future<void> initializeBackend() async {
  throw UnsupportedError(_message);
}

/// Creates a backend whose every member throws [UnsupportedError].
RawBox2D createRawBox2D() => RawBox2DUnsupported();

const _message =
    'forge2d is not supported on this platform yet. Native platforms are '
    'supported today; web support is planned for an upcoming release.';

/// Placeholder backend for unsupported platforms.
final class RawBox2DUnsupported implements RawBox2D {
  @override
  Object? noSuchMethod(Invocation invocation) =>
      throw UnsupportedError(_message);
}
