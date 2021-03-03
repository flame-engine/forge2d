class Pair implements Comparable<Pair> {
  final int proxyIdA;
  final int proxyIdB;

  const Pair(this.proxyIdA, this.proxyIdB);

  @override
  int compareTo(Pair pair2) {
    if (proxyIdA < pair2.proxyIdA) {
      return -1;
    }
    if (proxyIdA == pair2.proxyIdA) {
      return proxyIdB.compareTo(pair2.proxyIdB);
    }
    return 1;
  }
}
