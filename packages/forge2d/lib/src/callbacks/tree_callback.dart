import 'package:forge2d/forge2d.dart';

/// Callback for [DynamicTree]
abstract class TreeCallback {
  /// Callback from a query request.
  /// Return true if the query should be continued.
  bool treeCallback(int proxyId);
}
