/// The WebAssembly runtime plumbing of the web backend: module loading,
/// WASI stubs, host callback imports, and memory access.
///
/// Works under both dart2js and dart2wasm; only `dart:js_interop` is used.
library;

import 'dart:convert';
import 'dart:js_interop';
import 'dart:js_interop_unsafe';

@JS('fetch')
external JSPromise<_Response> _fetch(JSString url);

extension type _Response._(JSObject _) implements JSObject {
  external bool get ok;
  external JSPromise<JSArrayBuffer> arrayBuffer();
}

@JS('WebAssembly.instantiate')
external JSPromise<_InstantiateResult> _instantiate(
  JSArrayBuffer bytes,
  JSObject imports,
);

extension type _InstantiateResult._(JSObject _) implements JSObject {
  external _Instance get instance;
}

extension type _Instance._(JSObject _) implements JSObject {
  external JSObject get exports;
}

extension type _Memory._(JSObject _) implements JSObject {
  external JSArrayBuffer get buffer;
}

@JS('Float32Array')
extension type _F32._(JSObject _) implements JSObject {
  external _F32(JSArrayBuffer buffer);
  external double operator [](int index);
  external void operator []=(int index, double value);
}

@JS('Float64Array')
extension type _F64._(JSObject _) implements JSObject {
  external _F64(JSArrayBuffer buffer);
  external double operator [](int index);
  external void operator []=(int index, double value);
}

@JS('Int32Array')
extension type _I32._(JSObject _) implements JSObject {
  external _I32(JSArrayBuffer buffer);
  external int operator [](int index);
  external void operator []=(int index, int value);
}

@JS('Uint8Array')
extension type _U8._(JSObject _) implements JSObject {
  external _U8(JSArrayBuffer buffer);
  external int operator [](int index);
  external void operator []=(int index, int value);
  external int get length;
}

@JS('performance.now')
external double _performanceNow();

/// The host callbacks the shim can invoke synchronously during a call into
/// the module. Dispatch targets are swapped in by the backend around each
/// native call that can call back.
class WasmHostCallbacks {
  /// See `f2d_host_cast_ray` in f2d_shim.c.
  double Function(int, int, double, double, double, double, double)? castRay;

  /// See `f2d_host_overlap`.
  bool Function(int, int)? overlap;

  /// See `f2d_host_custom_filter`, keyed by nothing: the world's callback.
  bool Function(int, int, int, int)? customFilter;

  /// See `f2d_host_pre_solve`.
  bool Function(int, int, int, int, double, double)? preSolve;

  /// The debug draw dispatch target, non-null only during `worldDraw`.
  WasmDrawTarget? draw;
}

/// The debug draw callbacks, mirroring the seam's RawDebugDraw callbacks
/// but with vertex pointers resolved by the runtime.
class WasmDrawTarget {
  /// Creates a draw target.
  WasmDrawTarget({
    required this.drawPolygon,
    required this.drawSolidPolygon,
    required this.drawCircle,
    required this.drawSolidCircle,
    required this.drawSolidCapsule,
    required this.drawSegment,
    required this.drawTransform,
    required this.drawPoint,
    required this.drawString,
  });

  final void Function(List<double> vertices, int color) drawPolygon;
  final void Function(
    double,
    double,
    double,
    double,
    List<double> vertices,
    double radius,
    int color,
  )
  drawSolidPolygon;
  final void Function(double, double, double, int) drawCircle;
  final void Function(double, double, double, double, double, int)
  drawSolidCircle;
  final void Function(double, double, double, double, double, int)
  drawSolidCapsule;
  final void Function(double, double, double, double, int) drawSegment;
  final void Function(double, double, double, double) drawTransform;
  final void Function(double, double, double, int) drawPoint;
  final void Function(double, double, String, int) drawString;
}

/// A loaded Box2D WebAssembly module with typed memory access and a scratch
/// arena for argument and result passing.
class WasmRuntime {
  WasmRuntime._(this._exports, this._memory) {
    _refreshViews();
    _scratch = _malloc(_scratchSize);
  }

  final JSObject _exports;
  final _Memory _memory;

  /// The host callback dispatch table.
  final WasmHostCallbacks callbacks = WasmHostCallbacks();

  static const _scratchSize = 4096;

  late _F32 _f32;
  late _F64 _f64;
  late _I32 _i32;
  late _U8 _u8;
  int _bufferLength = 0;

  /// The fixed scratch arena, at least [_scratchSize] bytes.
  late final int _scratch;

  /// A growable secondary buffer for bulk data (events, vertex lists).
  int _bulk = 0;
  int _bulkCapacity = 0;

  final Map<String, JSFunction> _functions = {};

  /// Fetches and instantiates the module, trying each of [candidates] in
  /// order until one loads.
  static Future<WasmRuntime> load(List<Uri> candidates) async {
    Object? lastError;
    for (final candidate in candidates) {
      try {
        final response = await _fetch(candidate.toString().toJS).toDart;
        if (!response.ok) {
          lastError = StateError('HTTP error fetching $candidate');
          continue;
        }
        final bytes = await response.arrayBuffer().toDart;
        return await instantiate(bytes);
      } on Object catch (error) {
        lastError = error;
      }
    }
    throw StateError(
      'Could not load box2d.wasm from any of: ${candidates.join(', ')}. '
      'Pass the location explicitly: initializeForge2D(wasmUri: ...). '
      'Last error: $lastError',
    );
  }

  /// Instantiates the module from raw [bytes].
  static Future<WasmRuntime> instantiate(JSArrayBuffer bytes) async {
    late WasmRuntime runtime;

    final env = JSObject();
    void set(String name, JSFunction function) =>
        env.setProperty(name.toJS, function);

    set(
      'f2d_host_cast_ray',
      (
            JSNumber i1,
            JSNumber wg,
            JSNumber px,
            JSNumber py,
            JSNumber nx,
            JSNumber ny,
            JSNumber fraction,
          ) {
            final callback = runtime.callbacks.castRay;
            return (callback == null
                    ? 1.0
                    : callback(
                        i1.toDartInt,
                        wg.toDartInt,
                        px.toDartDouble,
                        py.toDartDouble,
                        nx.toDartDouble,
                        ny.toDartDouble,
                        fraction.toDartDouble,
                      ))
                .toJS;
          }
          .toJS,
    );
    set(
      'f2d_host_overlap',
      (JSNumber i1, JSNumber wg) {
        final callback = runtime.callbacks.overlap;
        final result = callback == null || callback(i1.toDartInt, wg.toDartInt);
        return (result ? 1 : 0).toJS;
      }.toJS,
    );
    set(
      'f2d_host_custom_filter',
      (JSNumber ai1, JSNumber awg, JSNumber bi1, JSNumber bwg) {
        final callback = runtime.callbacks.customFilter;
        final result =
            callback == null ||
            callback(
              ai1.toDartInt,
              awg.toDartInt,
              bi1.toDartInt,
              bwg.toDartInt,
            );
        return (result ? 1 : 0).toJS;
      }.toJS,
    );
    set(
      'f2d_host_pre_solve',
      (
            JSNumber ai1,
            JSNumber awg,
            JSNumber bi1,
            JSNumber bwg,
            JSNumber nx,
            JSNumber ny,
          ) {
            final callback = runtime.callbacks.preSolve;
            final result =
                callback == null ||
                callback(
                  ai1.toDartInt,
                  awg.toDartInt,
                  bi1.toDartInt,
                  bwg.toDartInt,
                  nx.toDartDouble,
                  ny.toDartDouble,
                );
            return (result ? 1 : 0).toJS;
          }
          .toJS,
    );
    set(
      'f2d_host_draw_polygon',
      (JSNumber vertices, JSNumber count, JSNumber color) {
        runtime.callbacks.draw?.drawPolygon(
          runtime.readF32List(vertices.toDartInt, count.toDartInt * 2),
          color.toDartInt,
        );
      }.toJS,
    );
    set(
      'f2d_host_draw_solid_polygon',
      (
            JSNumber px,
            JSNumber py,
            JSNumber qc,
            JSNumber qs,
            JSNumber vertices,
            JSNumber count,
            JSNumber radius,
            JSNumber color,
          ) {
            runtime.callbacks.draw?.drawSolidPolygon(
              px.toDartDouble,
              py.toDartDouble,
              qc.toDartDouble,
              qs.toDartDouble,
              runtime.readF32List(vertices.toDartInt, count.toDartInt * 2),
              radius.toDartDouble,
              color.toDartInt,
            );
          }
          .toJS,
    );
    set(
      'f2d_host_draw_circle',
      (JSNumber x, JSNumber y, JSNumber radius, JSNumber color) {
        runtime.callbacks.draw?.drawCircle(
          x.toDartDouble,
          y.toDartDouble,
          radius.toDartDouble,
          color.toDartInt,
        );
      }.toJS,
    );
    set(
      'f2d_host_draw_solid_circle',
      (
            JSNumber px,
            JSNumber py,
            JSNumber qc,
            JSNumber qs,
            JSNumber radius,
            JSNumber color,
          ) {
            runtime.callbacks.draw?.drawSolidCircle(
              px.toDartDouble,
              py.toDartDouble,
              qc.toDartDouble,
              qs.toDartDouble,
              radius.toDartDouble,
              color.toDartInt,
            );
          }
          .toJS,
    );
    set(
      'f2d_host_draw_solid_capsule',
      (
            JSNumber p1x,
            JSNumber p1y,
            JSNumber p2x,
            JSNumber p2y,
            JSNumber radius,
            JSNumber color,
          ) {
            runtime.callbacks.draw?.drawSolidCapsule(
              p1x.toDartDouble,
              p1y.toDartDouble,
              p2x.toDartDouble,
              p2y.toDartDouble,
              radius.toDartDouble,
              color.toDartInt,
            );
          }
          .toJS,
    );
    set(
      'f2d_host_draw_segment',
      (JSNumber p1x, JSNumber p1y, JSNumber p2x, JSNumber p2y, JSNumber color) {
        runtime.callbacks.draw?.drawSegment(
          p1x.toDartDouble,
          p1y.toDartDouble,
          p2x.toDartDouble,
          p2y.toDartDouble,
          color.toDartInt,
        );
      }.toJS,
    );
    set(
      'f2d_host_draw_transform',
      (JSNumber px, JSNumber py, JSNumber qc, JSNumber qs) {
        runtime.callbacks.draw?.drawTransform(
          px.toDartDouble,
          py.toDartDouble,
          qc.toDartDouble,
          qs.toDartDouble,
        );
      }.toJS,
    );
    set(
      'f2d_host_draw_point',
      (JSNumber x, JSNumber y, JSNumber size, JSNumber color) {
        runtime.callbacks.draw?.drawPoint(
          x.toDartDouble,
          y.toDartDouble,
          size.toDartDouble,
          color.toDartInt,
        );
      }.toJS,
    );
    set(
      'f2d_host_draw_string',
      (JSNumber x, JSNumber y, JSNumber text, JSNumber color) {
        runtime.callbacks.draw?.drawString(
          x.toDartDouble,
          y.toDartDouble,
          runtime.readCString(text.toDartInt),
          color.toDartInt,
        );
      }.toJS,
    );
    set(
      'emscripten_notify_memory_growth',
      (JSNumber index) {
        runtime._refreshViews();
      }.toJS,
    );

    final wasi = JSObject();
    // clock_time_get(clock_id, precision (i64), out_ptr) -> errno
    wasi.setProperty(
      'clock_time_get'.toJS,
      (JSNumber clockId, JSAny? precision, JSNumber outPointer) {
        final nanoseconds = _performanceNow() * 1e6;
        final low = (nanoseconds % 4294967296).floor();
        final high = (nanoseconds / 4294967296).floor();
        final index = outPointer.toDartInt ~/ 4;
        runtime._i32[index] = low;
        runtime._i32[index + 1] = high;
        return 0.toJS;
      }.toJS,
    );
    // fd_write(fd, iovs, iovs_len, nwritten_ptr) -> errno; swallow output.
    wasi.setProperty(
      'fd_write'.toJS,
      (JSNumber fd, JSNumber iovs, JSNumber iovsLength, JSNumber nwritten) {
        runtime._i32[nwritten.toDartInt ~/ 4] = 0;
        return 0.toJS;
      }.toJS,
    );
    // proc_exit(code) -> never; assert failures end up here.
    // ignore: prefer_function_declarations_over_variables
    final JSNumber Function(JSNumber) procExit = (code) {
      throw StateError('box2d.wasm exited with code ${code.toDartInt}');
    };
    wasi.setProperty('proc_exit'.toJS, procExit.toJS);

    final imports = JSObject()
      ..setProperty('env'.toJS, env)
      ..setProperty('wasi_snapshot_preview1'.toJS, wasi);

    final result = await _instantiate(bytes, imports).toDart;
    final exports = result.instance.exports;
    final memory = exports.getProperty('memory'.toJS)! as _Memory;
    return runtime = WasmRuntime._(exports, memory);
  }

  void _refreshViews() {
    final buffer = _memory.buffer;
    _f32 = _F32(buffer);
    _f64 = _F64(buffer);
    _i32 = _I32(buffer);
    _u8 = _U8(buffer);
    _bufferLength = _u8.length;
  }

  void _checkViews() {
    // Growth notifications cover emscripten-driven growth; this cheap check
    // covers anything else.
    if (_memory.buffer.byteLength != _bufferLength) {
      _refreshViews();
    }
  }

  int _malloc(int bytes) =>
      ((_functions['malloc'] ??=
                      _exports.getProperty('malloc'.toJS)! as JSFunction)
                  .callAsFunction(null, bytes.toJS)!
              as JSNumber)
          .toDartDouble
          .toInt();

  /// Calls an exported function with number arguments (bools as 0/1) and
  /// returns its number result, or 0 for void functions.
  num call(String name, List<num> arguments) {
    final function = _functions[name] ??=
        _exports.getProperty(name.toJS)! as JSFunction;
    final jsArguments = <JSAny?>[for (final a in arguments) a.toJS];
    final result = function.callAsFunctionVarArgs(null, jsArguments);
    _checkViews();
    return switch (result) {
      null => 0,
      final JSNumber number => number.toDartDouble,
      _ => 0,
    };
  }

  /// The fixed scratch arena pointer (4 KB).
  int get scratch => _scratch;

  /// Returns a pointer to at least [bytes] of bulk storage.
  int bulk(int bytes) {
    if (bytes > _bulkCapacity) {
      if (_bulk != 0) {
        (_functions['free'] ??=
                _exports.getProperty('free'.toJS)! as JSFunction)
            .callAsFunction(null, _bulk.toJS);
      }
      _bulkCapacity = bytes * 2;
      _bulk = _malloc(_bulkCapacity);
    }
    return _bulk;
  }

  // Typed reads and writes. Pointers are byte addresses.

  double readF32(int pointer) => _f32[pointer ~/ 4];
  double readF64(int pointer) => _f64[pointer ~/ 8];
  int readI32(int pointer) => _i32[pointer ~/ 4];

  void writeF32(int pointer, double value) => _f32[pointer ~/ 4] = value;
  void writeF64(int pointer, double value) => _f64[pointer ~/ 8] = value;
  void writeI32(int pointer, int value) => _i32[pointer ~/ 4] = value;

  List<double> readF32List(int pointer, int count) {
    final base = pointer ~/ 4;
    return [for (var i = 0; i < count; i++) _f32[base + i]];
  }

  List<double> readF64List(int pointer, int count) {
    final base = pointer ~/ 8;
    return [for (var i = 0; i < count; i++) _f64[base + i]];
  }

  List<int> readI32List(int pointer, int count) {
    final base = pointer ~/ 4;
    return [for (var i = 0; i < count; i++) _i32[base + i]];
  }

  void writeF32List(int pointer, List<double> values) {
    final base = pointer ~/ 4;
    for (var i = 0; i < values.length; i++) {
      _f32[base + i] = values[i];
    }
  }

  /// Writes [text] as a NUL-terminated UTF-8 string and returns the
  /// pointer, or 0 for null.
  int writeCString(int pointer, String? text) {
    if (text == null) {
      return 0;
    }
    final bytes = utf8.encode(text);
    for (var i = 0; i < bytes.length; i++) {
      _u8[pointer + i] = bytes[i];
    }
    _u8[pointer + bytes.length] = 0;
    return pointer;
  }

  /// Reads a NUL-terminated UTF-8 string.
  String readCString(int pointer) {
    if (pointer == 0) {
      return '';
    }
    final bytes = <int>[];
    var index = pointer;
    while (true) {
      final byte = _u8[index++];
      if (byte == 0) {
        break;
      }
      bytes.add(byte);
    }
    return utf8.decode(bytes);
  }
}

extension on JSFunction {
  JSAny? callAsFunctionVarArgs(JSAny? thisArg, List<JSAny?> arguments) =>
      callMethodVarArgs('apply'.toJS, [thisArg, arguments.toJS]);
}

extension on JSArrayBuffer {
  external int get byteLength;
}
