library box2d.buffer_utils;

import 'dart:typed_data';

/// Reallocate a buffer.
List<T> reallocateBufferWithAlloc<T>(
    List oldBuffer, int oldCapacity, int newCapacity, T alloc()) {
  assert(newCapacity > oldCapacity);
  List<T> newBuffer = List<T>(newCapacity);
  if (oldBuffer != null) {
    arrayCopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
  }
  for (int i = oldCapacity; i < newCapacity; i++) {
    try {
      newBuffer[i] = alloc();
    } catch (e) {
      throw "ReallocateBuffer Exception: $e";
    }
  }
  return newBuffer;
}

/// Reallocate a buffer.
List<int> reallocateBufferInt(
    List<int> oldBuffer, int oldCapacity, int newCapacity) {
  assert(newCapacity > oldCapacity);
  List<int> newBuffer = List<int>(newCapacity);
  if (oldBuffer != null) {
    arrayCopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
  }
  for (int i = oldCapacity; i < newCapacity; i++) {
    newBuffer[i] = 0;
  }
  return newBuffer;
}

/// Reallocate a buffer.
Float64List reallocateBuffer(
    Float64List oldBuffer, int oldCapacity, int newCapacity) {
  assert(newCapacity > oldCapacity);
  Float64List newBuffer = Float64List(newCapacity);
  if (oldBuffer != null) {
    arrayCopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
  }
  return newBuffer;
}

/// Reallocate a buffer. A 'deferred' buffer is reallocated only if it is not NULL.
/// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
List<T> reallocateBufferWithAllocDeferred<T>(
    List<T> buffer,
    int userSuppliedCapacity,
    int oldCapacity,
    int newCapacity,
    bool deferred,
    T alloc()) {
  assert(newCapacity > oldCapacity);
  assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity);
  if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
    buffer = reallocateBufferWithAlloc(buffer, oldCapacity, newCapacity, alloc);
  }
  return buffer;
}

/// Reallocate an int buffer. A 'deferred' buffer is reallocated only if it is not NULL.
/// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
List<int> reallocateBufferIntDeferred(List<int> buffer,
    int userSuppliedCapacity, int oldCapacity, int newCapacity, bool deferred) {
  assert(newCapacity > oldCapacity);
  assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity);
  if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
    buffer = reallocateBufferInt(buffer, oldCapacity, newCapacity);
  }
  return buffer;
}

/// Reallocate a float buffer. A 'deferred' buffer is reallocated only if it is not NULL.
/// If 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
Float64List reallocateBufferFloat64Deferred(Float64List buffer,
    int userSuppliedCapacity, int oldCapacity, int newCapacity, bool deferred) {
  assert(newCapacity > oldCapacity);
  assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity);
  if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
    buffer = reallocateBuffer(buffer, oldCapacity, newCapacity);
  }
  return buffer;
}

/// Rotate an array, see std::rotate
void rotate<T>(List<T> ray, int first, int newFirst, int last) {
  int next = newFirst;
  while (next != first) {
    var temp = ray[first];
    ray[first] = ray[next];
    ray[next] = temp;
    first++;
    next++;
    if (next == last) {
      next = newFirst;
    } else if (first == newFirst) {
      newFirst = next;
    }
  }
}

/// Helper function to allocate a list of integers and set all elements to 0.
List<int> intList(int size) => List<int>.filled(size, 0);

/// Helper function for ease of porting Java to Dart.
void arrayCopy(List src, int srcPos, List dest, int destPos, int length) {
  dest.setRange(destPos, length + destPos, src, srcPos);
}

// Replace Java's Arrays::sort.
// TODO(srdjan): Make a version that does not require copying.
void sort<T>(List<T> list, int fromPos, int toPos) {
  List<T> temp = List.from(list.getRange(fromPos, toPos));
  temp.sort();
  list.setRange(fromPos, toPos, temp);
}
