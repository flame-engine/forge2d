part of box2d;

class Vec2Array {
  final HashMap<int, List<Vector2>> _map = new HashMap<int, List<Vector2>>();

  List<Vector2> get(int argLength) {
    assert(argLength > 0);

    if (!_map.containsKey(argLength)) {
      _map[argLength] = getInitializedArray(argLength);
    }

    assert(_map[argLength].length ==
        argLength); // : "Array not built of correct length";
    return _map[argLength];
  }

  List<Vector2> getInitializedArray(int argLength) {
    final List<Vector2> ray = new List<Vector2>(argLength);
    for (int i = 0; i < ray.length; i++) {
      ray[i] = new Vector2.zero();
    }
    return ray;
  }
}
