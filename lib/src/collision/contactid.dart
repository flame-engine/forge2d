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
