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

enum ContactIDType { VERTEX, FACE }

class ContactID implements Comparable<ContactID> {
  Int8List _data = Int8List(4);

  void set indexA(int v) {
    _data[0] = v;
  }

  void set indexB(int v) {
    _data[1] = v;
  }

  void set typeA(int v) {
    _data[2] = v;
  }

  void set typeB(int v) {
    _data[3] = v;
    ;
  }

  int get indexA => _data[0];
  int get indexB => _data[1];
  int get typeA => _data[2];
  int get typeB => _data[3];

  int getKey() {
    return (indexA << 24) | (indexB << 16) | (typeA << 8) | (typeB);
  }

  bool isEqual(final ContactID cid) {
    return getKey() == cid.getKey();
  }

  ContactID();

  ContactID.copy(final ContactID c) {
    set(c);
  }

  void set(final ContactID c) {
    indexA = c.indexA;
    indexB = c.indexB;
    typeA = c.typeA;
    typeB = c.typeB;
  }

  void flip() {
    int tempA = indexA;
    indexA = indexB;
    indexB = tempA;
    tempA = typeA;
    typeA = typeB;
    typeB = tempA;
  }

  /// zeros out the data
  void zero() {
    indexA = 0;
    indexB = 0;
    typeA = 0;
    typeB = 0;
  }

  int compareTo(ContactID o) {
    return getKey() - o.getKey();
  }
}
