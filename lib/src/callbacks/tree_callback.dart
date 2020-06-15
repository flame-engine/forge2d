part of box2d;

/// Callback for {@link DynamicTree}
abstract class TreeCallback {
  /// Callback from a query request.
  /// @param proxyId the id of the proxy
  /// @return if the query should be continued
  bool treeCallback(int proxyId);
}
