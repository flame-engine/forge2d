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

library box2d.settings;

import 'dart:math' as Math;

const int INTEGER_MAX_VALUE = 0x3FFFFFFF;

/** A "close to zero" float epsilon value for use */
const double EPSILON = 1.1920928955078125E-7;

const double PI = Math.PI;

const int CONTACT_STACK_INIT_SIZE = 10;

// In Dart sinLUT is slwoer than using native Math.sin.
const bool SINCOS_LUT_ENABLED = false;
/**
   * smaller the precision, the larger the table. If a small table is used (eg, precision is .006 or
   * greater), make sure you set the table to lerp it's results. Accuracy chart is in the MathUtils
   * source. Or, run the tests yourself in {@link SinCosTest}.</br> </br> Good lerp precision
   * values:
   * <ul>
   * <li>.0092</li>
   * <li>.008201</li>
   * <li>.005904</li>
   * <li>.005204</li>
   * <li>.004305</li>
   * <li>.002807</li>
   * <li>.001508</li>
   * <li>9.32500E-4</li>
   * <li>7.48000E-4</li>
   * <li>8.47000E-4</li>
   * <li>.0005095</li>
   * <li>.0001098</li>
   * <li>9.50499E-5</li>
   * <li>6.08500E-5</li>
   * <li>3.07000E-5</li>
   * <li>1.53999E-5</li>
   * </ul>
   */
const double SINCOS_LUT_PRECISION = .00011;
final int SINCOS_LUT_LENGTH = (Math.PI * 2.0 / SINCOS_LUT_PRECISION).ceil();
/**
 * Use if the table's precision is large (eg .006 or greater). Although it is more expensive, it
 * greatly increases accuracy. Look in the MathUtils source for some test results on the accuracy
 * and speed of lerp vs non lerp. Or, run the tests yourself in {@link SinCosTest}.
 */
const bool SINCOS_LUT_LERP = false;

/**
 * The maximum number of contact points between two convex shapes.
 */
const int maxManifoldPoints = 2;

/**
 * The maximum number of vertices on a convex polygon.
 */
const int maxPolygonVertices = 8;

/**
 * This is used to fatten AABBs in the dynamic tree. This allows proxies to move by a small amount
 * without triggering a tree adjustment. This is in meters.
 */
const double aabbExtension = 0.1;

/**
 * This is used to fatten AABBs in the dynamic tree. This is used to predict the future position
 * based on the current displacement. This is a dimensionless multiplier.
 */
const double aabbMultiplier = 2.0;

/**
 * A small length used as a collision and constraint tolerance. Usually it is chosen to be
 * numerically significant, but visually insignificant.
 */
const double linearSlop = 0.005;

/**
 * A small angle used as a collision and constraint tolerance. Usually it is chosen to be
 * numerically significant, but visually insignificant.
 */
const double angularSlop = (2.0 / 180.0 * Math.PI);

/**
 * The radius of the polygon/edge shape skin. This should not be modified. Making this smaller
 * means polygons will have and insufficient for continuous collision. Making it larger may create
 * artifacts for vertex collision.
 */
const double polygonRadius = (2.0 * linearSlop);

/** Maximum number of sub-steps per contact in continuous physics simulation. */
const int maxSubSteps = 8;

// Dynamics

/**
   * Maximum number of contacts to be handled to solve a TOI island.
   */
const int maxTOIContacts = 32;

/**
 * A velocity threshold for elastic collisions. Any collision with a relative linear velocity
 * below this threshold will be treated as inelastic.
 */
const double velocityThreshold = 1.0;

/**
 * The maximum linear position correction used when solving constraints. This helps to prevent
 * overshoot.
 */
const double maxLinearCorrection = 0.2;

/**
 * The maximum angular position correction used when solving constraints. This helps to prevent
 * overshoot.
 */
const double maxAngularCorrection = (8.0 / 180.0 * Math.PI);

/**
 * The maximum linear velocity of a body. This limit is very large and is used to prevent
 * numerical problems. You shouldn't need to adjust this.
 */
const double maxTranslation = 2.0;
const double maxTranslationSquared = (maxTranslation * maxTranslation);

/**
 * The maximum angular velocity of a body. This limit is very large and is used to prevent
 * numerical problems. You shouldn't need to adjust this.
 */
const double maxRotation = (0.5 * Math.PI);
const double maxRotationSquared = (maxRotation * maxRotation);

/**
 * This scale factor controls how fast overlap is resolved. Ideally this would be 1 so that
 * overlap is removed in one time step. However using values close to 1 often lead to overshoot.
 */
const double baumgarte = 0.2;
const double toiBaugarte = 0.75;

// Sleep

/**
 * The time that a body must be still before it will go to sleep.
 */
const double timeToSleep = 0.5;

/**
 * A body cannot sleep if its linear velocity is above this tolerance.
 */
const double linearSleepTolerance = 0.01;

/**
 * A body cannot sleep if its angular velocity is above this tolerance.
 */
const double angularSleepTolerance = (2.0 / 180.0 * Math.PI);

// Particle

/**
 * A symbolic constant that stands for particle allocation error.
 */
const int invalidParticleIndex = (-1);

/**
 * The standard distance between particles, divided by the particle radius.
 */
const double particleStride = 0.75;

/**
 * The minimum particle weight that produces pressure.
 */
const double minParticleWeight = 1.0;

/**
 * The upper limit for particle weight used in pressure calculation.
 */
const double maxParticleWeight = 5.0;

/**
 * The maximum distance between particles in a triad, divided by the particle radius.
 */
const int maxTriadDistance = 2;
const int maxTriadDistanceSquared = (maxTriadDistance * maxTriadDistance);

/**
 * The initial size of particle data buffers.
 */
const int minParticleBufferCapacity = 256;
