part of box2d;

abstract class IOrderedStack<E> {
  /// Returns the next object in the pool
  E pop();

  /// Returns the next 'argNum' objects in the pool
  /// in an array
  /// @param argNum
  /// @return an array containing the next pool objects in
  ///       items 0-argNum.  Array length and uniqueness not
  ///       guaranteed.
  List<E> popSome(int argNum);

  /// Tells the stack to take back the last 'argNum' items
  void push(int argNum);
}
