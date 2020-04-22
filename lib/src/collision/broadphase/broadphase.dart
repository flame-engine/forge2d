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

abstract class BroadPhase {
  static const int NULL_PROXY = -1;

  /// Create a proxy with an initial AABB. Pairs are not reported until updatePairs is called.
  int createProxy(AABB aabb, Object userData);

  /// Destroy a proxy. It is up to the client to remove any pairs.
  void destroyProxy(int proxyId);

  /// Call MoveProxy as many times as you like, then when you are done call UpdatePairs to finalized
  /// the proxy pairs (for your time step).
  void moveProxy(int proxyId, AABB aabb, Vector2 displacement);

  void touchProxy(int proxyId);

  Object getUserData(int proxyId);

  AABB getFatAABB(int proxyId);

  bool testOverlap(int proxyIdA, int proxyIdB);

  /// Get the number of proxies.
  int getProxyCount();

  void drawTree(DebugDraw argDraw);

  /// Update the pairs. This results in pair callbacks. This can only add pairs.
  void updatePairs(PairCallback callback);

  /// Query an AABB for overlapping proxies. The callback class is called for each proxy that
  /// overlaps the supplied AABB.
  void query(TreeCallback callback, AABB aabb);

  /// Ray-cast against the proxies in the tree. This relies on the callback to perform a exact
  /// ray-cast in the case were the proxy contains a shape. The callback also performs the any
  /// collision filtering. This has performance roughly equal to k * log(n), where k is the number of
  /// collisions and n is the number of proxies in the tree.
  ///
  /// @param input the ray-cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
  /// @param callback a callback class that is called for each proxy that is hit by the ray.
  void raycast(TreeRayCastCallback callback, RayCastInput input);

  /// Get the height of the embedded tree.
  int getTreeHeight();

  int getTreeBalance();

  double getTreeQuality();
}
