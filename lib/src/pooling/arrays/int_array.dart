part of box2d;

class IntArray {
  final HashMap<int, List<int>> _map = new HashMap<int, List<int>>();

  List<int> get(int argLength) {
    assert(argLength > 0);

    if (!_map.containsKey(argLength)) {
      _map[argLength] = getInitializedArray(argLength);
    }

    assert(_map[argLength].length ==
        argLength); // : "Array not built of correct length";
    return _map[argLength];
  }

  List<int> getInitializedArray(int argLength) {
    return BufferUtils.allocClearIntList(argLength);
  }
}
