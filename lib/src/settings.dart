import 'dart:math';

/// NOTE: If you change any of these values, do make sure that you know what you
/// are doing. Don't change these because your bodies are moving to slowly,
/// then you have done something else that is wrong, like setting the scale too
/// low.

int intMaxValue = 0x3FFFFFFF;

/// A "close to zero" float epsilon value for use
double epsilon = 1.1920928955078125E-7;

/// The maximum number of contact points between two convex shapes.
int maxManifoldPoints = 2;

/// The maximum number of vertices on a convex polygon.
int maxPolygonVertices = 8;

/// This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
/// without triggering a tree adjustment. This is in meters.
double aabbExtension = 0.1;

/// This is used to fatten AABBs in the dynamic tree. This is used to predict the future position
/// based on the current displacement. This is a dimensionless multiplier.
double aabbMultiplier = 2.0;

/// A small length used as a collision and constraint tolerance. Usually it is chosen to be
/// numerically significant, but visually insignificant.
double linearSlop = 0.005;

/// A small angle used as a collision and constraint tolerance. Usually it is chosen to be
/// numerically significant, but visually insignificant.
double angularSlop = 2.0 / 180.0 * pi;

/// The radius of the polygon/edge shape skin. This should not be modified. Making this smaller
/// means polygons will have and insufficient for continuous collision. Making it larger may create
/// artifacts for vertex collision.
double polygonRadius = 2.0 * linearSlop;

/// Maximum number of sub-steps per contact in continuous physics simulation.
int maxSubSteps = 8;

// Dynamics

/// Maximum number of contacts to be handled to solve a TOI island.
int maxTOIContacts = 32;

/// A velocity threshold for elastic collisions. Any collision with a relative linear velocity
/// below this threshold will be treated as inelastic.
double velocityThreshold = 1.0;

/// The maximum linear position correction used when solving constraints. This helps to prevent
/// overshoot.
double maxLinearCorrection = 0.2;

/// The maximum angular position correction used when solving constraints. This helps to prevent
/// overshoot.
double maxAngularCorrection = 8.0 / 180.0 * pi;

/// The maximum linear velocity of a body. This limit is very large and is used to prevent
/// numerical problems. You shouldn't need to adjust this.
double maxTranslation = 2.0;
double maxTranslationSquared = maxTranslation * maxTranslation;

/// The maximum angular velocity of a body. This limit is very large and is used to prevent
/// numerical problems. You shouldn't need to adjust this.
double maxRotation = 0.5 * pi;
double maxRotationSquared = maxRotation * maxRotation;

/// This scale factor controls how fast overlap is resolved. Ideally this would be 1 so that
/// overlap is removed in one time step. However using values close to 1 often lead to overshoot.
double baumgarte = 0.2;

// Sleep

/// The time that a body must be still before it will go to sleep.
double timeToSleep = 0.5;

/// A body cannot sleep if its linear velocity is above this tolerance.
double linearSleepTolerance = 0.01;

/// A body cannot sleep if its angular velocity is above this tolerance.
double angularSleepTolerance = 2.0 / 180.0 * pi;

// Particle

/// A symbolic constant that stands for particle allocation error.
int invalidParticleIndex = -1;

/// The standard distance between particles, divided by the particle radius.
double particleStride = 0.75;

/// The minimum particle weight that produces pressure.
double minParticleWeight = 1.0;

/// The upper limit for particle weight used in pressure calculation.
double maxParticleWeight = 5.0;

/// The maximum distance between particles in a triad, divided by the particle radius.
int maxTriadDistance = 2;
int maxTriadDistanceSquared = maxTriadDistance * maxTriadDistance;

/// The initial size of particle data buffers.
int minParticleBufferCapacity = 256;

/// The amount of iterations for velocities that should be done in the solver.
int velocityIterations = 10;

/// The amount of iterations for positions that should be done in the solver.
int positionIterations = 10;
