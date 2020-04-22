/// *****************************************************************************
/// Copyright (c) 2015, Daniel Murphy, Google
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without modification,
/// are permitted provided that the following conditions are met:
///  * Redistributions of source code must retain the above copyright notice,
///    this list of conditions and the following disclaimer.
///  * Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
/// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
/// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
/// IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
/// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
/// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
/// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
/// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
/// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
/// POSSIBILITY OF SUCH DAMAGE.
/// *****************************************************************************

part of box2d;

/// A fixture definition is used to create a fixture. This class defines an abstract fixture
/// definition. You can reuse fixture definitions safely.
class FixtureDef {
  /// The shape, this must be set. The shape will be cloned, so you can create the shape on the
  /// stack.
  Shape shape = null;

  /// Use this to store application specific fixture data.
  Object userData;

  /// The friction coefficient, usually in the range [0,1].
  double friction = 0.2;

  /// The restitution (elasticity) usually in the range [0,1].
  double restitution = 0.0;

  /// The density, usually in kg/m^2
  double density = 0.0;

  /// A sensor shape collects contact information but never generates a collision response.
  bool isSensor = false;

  /// Contact filtering data;
  Filter filter = new Filter();

  /// The shape, this must be set. The shape will be cloned, so you can create the shape on the
  /// stack.
  Shape getShape() {
    return shape;
  }

  /// The shape, this must be set. The shape will be cloned, so you can create the shape on the
  /// stack.
  void setShape(Shape shape) {
    this.shape = shape;
  }

  /// Use this to store application specific fixture data.
  Object getUserData() {
    return userData;
  }

  /// Use this to store application specific fixture data.
  void setUserData(Object userData) {
    this.userData = userData;
  }

  /// The friction coefficient, usually in the range [0,1].
  double getFriction() {
    return friction;
  }

  /// The friction coefficient, usually in the range [0,1].
  void setFriction(double friction) {
    this.friction = friction;
  }

  /// The restitution (elasticity) usually in the range [0,1].
  double getRestitution() {
    return restitution;
  }

  /// The restitution (elasticity) usually in the range [0,1].
  void setRestitution(double restitution) {
    this.restitution = restitution;
  }

  /// The density, usually in kg/m^2
  double getDensity() {
    return density;
  }

  /// The density, usually in kg/m^2
  void setDensity(double density) {
    this.density = density;
  }

  /// Contact filtering data;
  Filter getFilter() {
    return filter;
  }

  /// Contact filtering data;
  void setFilter(Filter filter) {
    this.filter = filter;
  }
}
