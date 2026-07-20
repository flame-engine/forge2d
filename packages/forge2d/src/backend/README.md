# The backend seam

`RawBox2D` (raw_box2d.dart) is the only doorway between the public API layer
in `lib/src/api/` and the Box2D library. It exists so that the exact same
public API can run on two very different transport mechanisms:

- **Native** (`raw_box2d_ffi.dart`): dart:ffi calls into the library built by
  `hook/build.dart`, using the generated bindings in `lib/src/ffi/`.
- **Web** (future): dart:js_interop calls into a WebAssembly build of Box2D
  through a pointer-based C shim.

The concrete implementation is chosen at compile time by the conditional
export in `backend.dart`; on unsupported platforms a stub that throws
`UnsupportedError` is used.

## Contract rules

Breaking these rules is cheap to do today and expensive to undo once the web
backend exists, so they are enforced in review:

1. **Primitives only.** Interface signatures may use `int`, `double`, `bool`,
   `String`, records of those, and typed-data lists. Never `dart:ffi` or
   `dart:js_interop` types, and no Dart classes from the API layer.
2. **No 64-bit integers.** dart2js cannot represent them. Ids are packed into
   32-bit safe ints:
   - world id: `index1 | (generation << 16)` (both are 16-bit natively);
   - body/shape/chain/joint ids: a pair `(index1, world0 | (generation << 16))`
     passed as two separate `int` parameters and returned as an
     `(int, int)` record.
   64-bit bit fields (collision filter masks) cross the seam as `int` and must
   be written with sign-extension semantics: `-1` means all bits set.
3. **No callbacks across the seam** unless there is no alternative. Events are
   polled, queries return their results in full. The web backend cannot pass
   function pointers into wasm without significant cost.
4. **Def structs cross as flattened named parameters.** Implementations start
   from the native `b2Default*Def()` values and overwrite every field the
   method receives, which also takes care of `internalValue` cookies.
5. **The API layer owns all Dart objects.** User data never crosses the seam;
   the native user data pointers stay untouched.
