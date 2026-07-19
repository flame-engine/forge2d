# Benchmark for Forge2D

The classic [bench2d](https://github.com/joelgwebber/bench2d) benchmark:
a 40-high pyramid of boxes, 256 warm-up frames followed by 256 measured
frames.

```sh
dart run bin/bench2d.dart
```

Or from the repository root:

```sh
melos benchmark
```

The web harness returns when the WebAssembly backend lands.
