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

library box2d.common.canvas_viewport_transform;

import 'dart:html';

import 'package:box2d_flame/box2d.dart';

/// Transform for drawing using a canvas context. Y-flip is permenantly set
/// to true.
class CanvasViewportTransform extends ViewportTransform {
  static const double DEFAULT_DRAWING_SCALE = 20.0;

  /// Constructs a new viewport transform with the default scale.
  CanvasViewportTransform(Vector2 _extents, Vector2 _center)
      : super(_extents, _center, DEFAULT_DRAWING_SCALE) {
    yFlip = true;
  }

  /// Sets the rendering context such that all drawing commands given in terms
  /// of the world coordinate system will display correctly on the canvas screen.
  void updateTransformation(CanvasRenderingContext2D ctx) {
    // Clear all previous transformation.
    ctx.setTransform(1, 0, 0, 1, 0, 0);

    // Translate to the center of the canvas screen. This will be considered the
    // actual origin.
    ctx.translate(extents.x, extents.y);

    // Translate to account for the currently applied translation.
    ctx.translate(translation.x, translation.y);

    // Scale everything according to the current scale and mirror the y-axis.
    ctx.scale(scale, -scale);
  }
}
