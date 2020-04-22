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

part of box2d.common;

class ViewportTransform {
  ViewportTransform(Vector2 e, Vector2 c, this.scale)
      : extents = new Vector2.copy(e),
        center = new Vector2.copy(c);

  /// if we flip the y axis when transforming.
  bool yFlip;

  /// This is the half-width and half-height.
  /// This should be the actual half-width and
  /// half-height, not anything transformed or scaled.
  Vector2 extents;

  /// Returns the scaling factor used in converting from world sizes to rendering
  /// sizes.
  double scale;

  /// center of the viewport.
  Vector2 center;

  /// Sets the transform's center to the given x and y coordinates,
  /// and using the given scale.
  void setCamera(double x, double y, double s) {
    center.setValues(x, y);
    scale = s;
  }

  /// The current translation is the difference in canvas units between the
  /// actual center of the canvas and the currently specified center. For
  /// example, if the actual canvas center is (5, 5) but the current center is
  /// (6, 6), the translation is (1, 1).
  Vector2 get translation {
    Vector2 result = new Vector2.copy(extents);
    return result..sub(center);
  }

  void set translation(Vector2 translation) {
    center.setFrom(extents);
    center.sub(translation);
  }

  /// Takes the world coordinates and return the corresponding screen coordinates
  Vector2 getWorldToScreen(Vector2 argWorld) {
    // Correct for canvas considering the upper-left corner, rather than the
    // center, to be the origin.
    double gridCorrectedX = (argWorld.x * scale) + extents.x;
    double gridCorrectedY = extents.y - (argWorld.y * scale);

    return Vector2(
        gridCorrectedX + translation.x, gridCorrectedY + -translation.y);
  }

  /// Takes the screen coordinates and return the corresponding world coordinates
  Vector2 getScreenToWorld(Vector2 argScreen) {
    double translationCorrectedX = argScreen.x - translation.x;
    double translationCorrectedY = argScreen.y + translation.y;

    double gridCorrectedX = (translationCorrectedX - extents.x) / scale;
    double gridCorrectedY = ((translationCorrectedY - extents.y) * -1) / scale;
    return Vector2(gridCorrectedX, gridCorrectedY);
  }
}
