class Pair implements Comparable<Pair> {
  int proxyIdA = 0;
  int proxyIdB = 0;

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
