/*******************************************************************************
 * Copyright (c) 2015, Daniel Murphy, Google
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

library box2d.buffer_utils;

import 'dart:typed_data';

/** Reallocate a buffer. */
List<T> reallocateBufferWithAlloc<T>(
    List oldBuffer, int oldCapacity, int newCapacity, T alloc()) {
  assert(newCapacity > oldCapacity);
  List<T> newBuffer = new List<T>(newCapacity);
  if (oldBuffer != null) {
    arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
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

/** Reallocate a buffer. */
List<int> reallocateBufferInt(
    List<int> oldBuffer, int oldCapacity, int newCapacity) {
  assert(newCapacity > oldCapacity);
  List<int> newBuffer = new List<int>(newCapacity);
  if (oldBuffer != null) {
    arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
  }
  for (int i = oldCapacity; i < newCapacity; i++) {
    newBuffer[i] = 0;
  }
  return newBuffer;
}

/** Reallocate a buffer. */
Float64List reallocateBuffer(
    Float64List oldBuffer, int oldCapacity, int newCapacity) {
  assert(newCapacity > oldCapacity);
  Float64List newBuffer = new Float64List(newCapacity);
  if (oldBuffer != null) {
    arraycopy(oldBuffer, 0, newBuffer, 0, oldCapacity);
  }
  return newBuffer;
}

/**
   * Reallocate a buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
   * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
   */
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

/**
   * Reallocate an int buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
   * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
   */
List<int> reallocateBufferIntDeferred(List<int> buffer,
    int userSuppliedCapacity, int oldCapacity, int newCapacity, bool deferred) {
  assert(newCapacity > oldCapacity);
  assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity);
  if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
    buffer = reallocateBufferInt(buffer, oldCapacity, newCapacity);
  }
  return buffer;
}

/**
   * Reallocate a float buffer. A 'deferred' buffer is reallocated only if it is not NULL. If
   * 'userSuppliedCapacity' is not zero, buffer is user supplied and must be kept.
   */
Float64List reallocateBufferFloat64Deferred(Float64List buffer,
    int userSuppliedCapacity, int oldCapacity, int newCapacity, bool deferred) {
  assert(newCapacity > oldCapacity);
  assert(userSuppliedCapacity == 0 || newCapacity <= userSuppliedCapacity);
  if ((!deferred || buffer != null) && userSuppliedCapacity == 0) {
    buffer = reallocateBuffer(buffer, oldCapacity, newCapacity);
  }
  return buffer;
}

/** Rotate an array, see std::rotate */
void rotate<T>(List<T> ray, int first, int new_first, int last) {
  int next = new_first;
  while (next != first) {
    var temp = ray[first];
    ray[first] = ray[next];
    ray[next] = temp;
    first++;
    next++;
    if (next == last) {
      next = new_first;
    } else if (first == new_first) {
      new_first = next;
    }
  }
}

/** Helper function to allocate a list of integers and set all elements to 0. */
List<int> allocClearIntList(int size) => new List<int>.filled(size, 0);

/**
 * Helper function for ease of porting Java to Dart.
 */
void arraycopy(List src, int srcPos, List dest, int destPos, int length) {
  dest.setRange(destPos, length + destPos, src, srcPos);
}

// Replace Java's Arrays::sort.
// TODO(srdjan): Make a version that does not require copying.
void sort<T>(List<T> list, int fromPos, int toPos) {
  List<T> temp = new List.from(list.getRange(fromPos, toPos));
  temp.sort();
  list.setRange(fromPos, toPos, temp);
}
