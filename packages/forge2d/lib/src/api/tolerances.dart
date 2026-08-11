import 'package:forge2d/src/initialize.dart';

/// The Box2D tolerances that scale with the length unit.
///
/// Box2D is tuned for meters, kilograms and seconds, and a handful of its
/// tolerances are absolute lengths rather than fractions of the objects they
/// apply to. If a world is laid out at a much smaller scale than a meter,
/// those tolerances stop being "visually insignificant" and start dominating
/// the simulation: shapes report contacts before they touch, and the
/// broadphase margin grows larger than the shapes themselves.
///
/// The usual fix is to lay the world out so that moving objects are roughly
/// 0.1 to 10 meters, with 1 meter being the sweet spot. When that is not
/// possible, tell Box2D what a meter means in your units with
/// `initializeForge2D(lengthUnitsPerMeter: ...)`, and every value here moves
/// with it.
///
/// These mirror the constants in `src/constants.h` of Box2D v3.1.1. There are
/// further absolute thresholds that the length unit scales but that are
/// per-world or per-body rather than global, so they are fields on the
/// definitions instead: `WorldDef.restitutionThreshold`,
/// `WorldDef.hitEventThreshold`, `WorldDef.maxContactPushSpeed`,
/// `WorldDef.maximumLinearSpeed` and `BodyDef.sleepThreshold`.
abstract final class Tolerances {
  /// How many length units make up one meter, mirroring
  /// `b2GetLengthUnitsPerMeter`.
  ///
  /// Defaults to 1, meaning that forge2d lengths are meters. Set it through
  /// `initializeForge2D(lengthUnitsPerMeter: ...)`.
  static double get lengthUnitsPerMeter => rawBox2D.getLengthUnitsPerMeter();

  /// The collision and constraint tolerance, `0.005` of a meter.
  ///
  /// Shapes are allowed to overlap by this much so that contacts stay stable.
  static double get linearSlop => 0.005 * lengthUnitsPerMeter;

  /// The separation at which shapes start reporting contacts, `0.02` of a
  /// meter, or four times the [linearSlop].
  ///
  /// Box2D creates contact points for shapes that are approaching but not yet
  /// touching, which is what keeps fast objects from passing through each
  /// other and removes most collision jitter. It also means that
  /// `beginContact` fires while there is still a visible gap, so shapes that
  /// are not comfortably larger than this behave as if they were permanently
  /// in contact.
  static double get speculativeDistance => 4 * linearSlop;

  /// How much the broadphase fattens shape bounds, `0.05` of a meter.
  ///
  /// Lets a shape move a little without the dynamic tree having to be
  /// rebuilt.
  static double get aabbMargin => 0.05 * lengthUnitsPerMeter;
}
