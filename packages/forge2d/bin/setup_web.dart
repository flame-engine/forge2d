// Copies the bundled box2d.wasm into a web app's web/ directory, where
// initializeForge2D() finds it at runtime.
//
// Run from the root of your app:
//   dart run forge2d:setup_web
// ignore_for_file: avoid_print
import 'dart:io';
import 'dart:isolate';

Future<void> main() async {
  final packageUri = await Isolate.resolvePackageUri(
    Uri.parse('package:forge2d/src/backend/wasm/box2d.wasm'),
  );
  if (packageUri == null) {
    stderr.writeln('Could not resolve the forge2d package.');
    exitCode = 1;
    return;
  }

  final webDirectory = Directory('web');
  if (!webDirectory.existsSync()) {
    stderr.writeln(
      'No web/ directory found. Run this from the root of a web app.',
    );
    exitCode = 1;
    return;
  }

  final target = File('web/box2d.wasm');
  File.fromUri(packageUri).copySync(target.path);
  print('Copied box2d.wasm to ${target.path}');
}
