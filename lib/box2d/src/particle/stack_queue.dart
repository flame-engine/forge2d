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

part of box2d;


class StackQueue<T> {

  List<T> _m_buffer;
  int _m_front = 0;
  int _m_back = 0;
  int _m_end = 0;

  StackQueue() {}

  void reset(List<T> buffer) {
    _m_buffer = buffer;
    _m_front = 0;
    _m_back = 0;
    _m_end = buffer.length;
  }

  void push(T task) {
    if (_m_back >= _m_end) {
      Settings.arraycopy(_m_buffer, _m_front, _m_buffer, 0, _m_back - _m_front);
      _m_back -= _m_front;
      _m_front = 0;
      if (_m_back >= _m_end) {
        return;
      }
    }
    _m_buffer[_m_back++] = task;
  }

  T pop() {
    assert (_m_front < _m_back);
    return _m_buffer[_m_front++];
  }

  bool empty() {
    return _m_front >= _m_back;
  }

  T front() {
    return _m_buffer[_m_front];
  }
}
