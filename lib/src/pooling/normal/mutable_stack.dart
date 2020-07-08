part of box2d;

// TODO(spydon): refactor to just use a normal stack
abstract class MutableStack<E> implements IDynamicStack<E> {
  List<E> _stack;
  int _index;
  int _size;

  MutableStack(int argInitSize) {
    _index = 0;
    _stack = null;
    _index = 0;
    _size = 0;
    extendStack(argInitSize);
  }

  void extendStack(int argSize) {
    List<E> newStack = newArray(argSize);
    if (_stack != null) {
      BufferUtils.arrayCopy(_stack, 0, newStack, 0, _size);
    }
    for (int i = 0; i < newStack.length; i++) {
      newStack[i] = newInstance();
    }
    _stack = newStack;
    _size = newStack.length;
  }

  E pop() {
    if (_index >= _size) {
      extendStack(_size * 2);
    }
    return _stack[_index++];
  }

  void push(E argObject) {
    assert(_index > 0);
    _stack[--_index] = argObject;
  }

  /// Creates a new instance of the object contained by this stack.
  E newInstance();

  List<E> newArray(int size) {
    return List<E>(size);
  }
}
