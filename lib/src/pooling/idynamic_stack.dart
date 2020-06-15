part of box2d;

abstract class IDynamicStack<E> {
  /// Pops an item off the stack
  /// @return
  E pop();

  /// Pushes an item back on the stack
  /// @param argObject
  void push(E argObject);
}
