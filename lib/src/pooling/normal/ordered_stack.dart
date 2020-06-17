part of box2d;

abstract class OrderedStack<E> {
  final List<E> _pool;
  int _index;
  final int _size;
  final List<E> _container;

  OrderedStack(int argStackSize, int argContainerSize)
      : _pool = List<E>(argStackSize),
        _index = 0,
        _size = argStackSize,
        _container = List<E>(argContainerSize) {
    for (int i = 0; i < argStackSize; i++) {
      _pool[i] = newInstance();
    }
  }

  E pop() {
    assert(_index <
        _size); // "End of stack reached, there is probably a leak somewhere";
    return _pool[_index++];
  }

  List<E> popSome(int argNum) {
    assert(_index + argNum <
        _size); // "End of stack reached, there is probably a leak somewhere";
    assert(argNum <= _container.length); // "Container array is too small";
    BufferUtils.arrayCopy(_pool, _index, _container, 0, argNum);
    _index += argNum;
    return _container;
  }

  void push(int argNum) {
    _index -= argNum;
    assert(_index >= 0);
  }

  /// Creates a new instance of the object contained by this stack.
  E newInstance();
}
