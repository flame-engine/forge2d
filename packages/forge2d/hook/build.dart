import 'package:code_assets/code_assets.dart';
import 'package:hooks/hooks.dart';
import 'package:logging/logging.dart';
import 'package:native_toolchain_c/native_toolchain_c.dart';

/// Box2D C source files, relative to the package root.
///
/// Listed explicitly so that publishing and hook dependency tracking stay
/// deterministic. Regenerate with tool/vendor_box2d.sh when upgrading.
const _sources = [
  'third_party/box2d/src/aabb.c',
  'third_party/box2d/src/arena_allocator.c',
  'third_party/box2d/src/array.c',
  'third_party/box2d/src/bitset.c',
  'third_party/box2d/src/body.c',
  'third_party/box2d/src/broad_phase.c',
  'third_party/box2d/src/constraint_graph.c',
  'third_party/box2d/src/contact.c',
  'third_party/box2d/src/contact_solver.c',
  'third_party/box2d/src/core.c',
  'third_party/box2d/src/distance.c',
  'third_party/box2d/src/distance_joint.c',
  'third_party/box2d/src/dynamic_tree.c',
  'third_party/box2d/src/geometry.c',
  'third_party/box2d/src/hull.c',
  'third_party/box2d/src/id_pool.c',
  'third_party/box2d/src/island.c',
  'third_party/box2d/src/joint.c',
  'third_party/box2d/src/manifold.c',
  'third_party/box2d/src/math_functions.c',
  'third_party/box2d/src/motor_joint.c',
  'third_party/box2d/src/mouse_joint.c',
  'third_party/box2d/src/mover.c',
  'third_party/box2d/src/prismatic_joint.c',
  'third_party/box2d/src/revolute_joint.c',
  'third_party/box2d/src/sensor.c',
  'third_party/box2d/src/shape.c',
  'third_party/box2d/src/solver.c',
  'third_party/box2d/src/solver_set.c',
  'third_party/box2d/src/table.c',
  'third_party/box2d/src/timer.c',
  'third_party/box2d/src/types.c',
  'third_party/box2d/src/weld_joint.c',
  'third_party/box2d/src/wheel_joint.c',
  'third_party/box2d/src/world.c',
];

void main(List<String> args) async {
  await build(args, (input, output) async {
    if (!input.config.buildCodeAssets) {
      return;
    }

    final code = input.config.code;
    final os = code.targetOS;
    final architecture = code.targetArchitecture;
    // Box2D auto-selects SSE2 on x64 and NEON on arm64; everything else
    // needs the scalar fallback.
    final hasSimd =
        architecture == Architecture.x64 || architecture == Architecture.arm64;

    final builder = CBuilder.library(
      name: 'box2d',
      assetName: 'src/ffi/box2d.g.dart',
      sources: _sources,
      includes: ['third_party/box2d/include', 'third_party/box2d/src'],
      std: 'c17',
      optimizationLevel: OptimizationLevel.o2,
      defines: {
        // Mark the public API as exported; without this MSVC builds a DLL
        // with no visible symbols.
        'box2d_EXPORTS': '1',
        if (!hasSimd) 'BOX2D_DISABLE_SIMD': '1',
        // Expose POSIX declarations (clock_gettime) that strict -std=c17
        // hides in glibc/bionic headers.
        if (os == OS.linux || os == OS.android) '_DEFAULT_SOURCE': '1',
      },
      libraries: [
        if (os == OS.linux || os == OS.android) 'm',
      ],
    );

    await builder.run(
      input: input,
      output: output,
      logger: Logger('')
        ..level = Level.ALL
        // ignore: avoid_print, hook output only surfaces when the build fails.
        ..onRecord.listen((record) => print(record.message)),
    );
  });
}
