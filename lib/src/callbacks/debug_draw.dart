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

/**
 * Implement this abstract class to allow DBox2d to automatically draw your physics for debugging
 * purposes. Not intended to replace your own custom rendering routines!
 * 
 * @author Daniel Murphy
 */
abstract class DebugDraw {

  /** Draw shapes */
  static const int e_shapeBit = 1 << 1;
  /** Draw joint connections */
  static const int e_jointBit = 1 << 2;
  /** Draw axis aligned bounding boxes */
  static const int e_aabbBit = 1 << 3;
  /** Draw pairs of connected objects */
  static const int e_pairBit = 1 << 4;
  /** Draw center of mass frame */
  static const int e_centerOfMassBit = 1 << 5;
  /** Draw dynamic tree */
  static const int e_dynamicTreeBit = 1 << 6;
  /** Draw only the wireframe for drawing performance */
  static const int e_wireframeDrawingBit = 1 << 7;

  int m_drawFlags = 0;
  IViewportTransform viewportTransform;

  DebugDraw.zero();
  DebugDraw(this.viewportTransform);

  void setViewportTransform(IViewportTransform viewportTransform) {
    this.viewportTransform = viewportTransform;
  }

  void setFlags(int flags) {
    m_drawFlags = flags;
  }

  int getFlags() {
    return m_drawFlags;
  }

  void appendFlags(int flags) {
    m_drawFlags |= flags;
  }

  void clearFlags(int flags) {
    m_drawFlags &= ~flags;
  }

  /**
   * Draw a closed polygon provided in CCW order. This implementation uses
   * {@link #drawSegment(Vec2, Vec2, Color3f)} to draw each side of the polygon.
   * 
   * @param vertices
   * @param vertexCount
   * @param color
   */
  void drawPolygon(List<Vec2> vertices, int vertexCount, Color3i color) {
    if (vertexCount == 1) {
      drawSegment(vertices[0], vertices[0], color);
      return;
    }

    for (int i = 0; i < vertexCount - 1; i += 1) {
      drawSegment(vertices[i], vertices[i + 1], color);
    }

    if (vertexCount > 2) {
      drawSegment(vertices[vertexCount - 1], vertices[0], color);
    }
  }

  void drawPoint(Vec2 argPoint, double argRadiusOnScreen, Color3i argColor);

  /**
   * Draw a solid closed polygon provided in CCW order.
   * 
   * @param vertices
   * @param vertexCount
   * @param color
   */
  void drawSolidPolygon(List<Vec2> vertices, int vertexCount, Color3i color);

  /**
   * Draw a circle.
   * 
   * @param center
   * @param radius
   * @param color
   */
  void drawCircle(Vec2 center, double radius, Color3i color);

  /** Draws a circle with an axis */
  void drawCircleAxis(Vec2 center, double radius, Vec2 axis, Color3i color) {
    drawCircle(center, radius, color);
  }

  /**
   * Draw a solid circle.
   * 
   * @param center
   * @param radius
   * @param axis
   * @param color
   */
  void drawSolidCircle(Vec2 center, double radius, Vec2 axis, Color3i color);

  /**
   * Draw a line segment.
   * 
   * @param p1
   * @param p2
   * @param color
   */
  void drawSegment(Vec2 p1, Vec2 p2, Color3i color);

  /**
   * Draw a transform. Choose your own length scale
   * 
   * @param xf
   */
  void drawTransform(Transform xf);

  /**
   * Draw a string.
   * 
   * @param x
   * @param y
   * @param s
   * @param color
   */
  void drawStringXY(double x, double y, String s, Color3i color);

  /**
   * Draw a particle array
   * 
   * @param colors can be null
   */
  void drawParticles(
      List<Vec2> centers, double radius, List<ParticleColor> colors, int count);

  /**
   * Draw a particle array
   * 
   * @param colors can be null
   */
  void drawParticlesWireframe(
      List<Vec2> centers, double radius, List<ParticleColor> colors, int count);

  /** Called at the end of drawing a world */
  void flush() {}

  void drawString(Vec2 pos, String s, Color3i color) {
    drawStringXY(pos.x, pos.y, s, color);
  }

  IViewportTransform getViewportTranform() {
    return viewportTransform;
  }

  /**
   * @param x
   * @param y
   * @param scale
   * @deprecated use the viewport transform in {@link #getViewportTranform()}
   */
  void setCamera(double x, double y, double scale) {
    viewportTransform.setCamera(x, y, scale);
  }

  /**
   * @param argScreen
   * @param argWorld
   */
  void getScreenToWorldToOut(Vec2 argScreen, Vec2 argWorld) {
    viewportTransform.getScreenToWorld(argScreen, argWorld);
  }

  /**
   * @param argWorld
   * @param argScreen
   */
  void getWorldToScreenToOut(Vec2 argWorld, Vec2 argScreen) {
    viewportTransform.getWorldToScreen(argWorld, argScreen);
  }

  /**
   * Takes the world coordinates and puts the corresponding screen coordinates in argScreen.
   * 
   * @param worldX
   * @param worldY
   * @param argScreen
   */
  void getWorldToScreenToOutXY(double worldX, double worldY, Vec2 argScreen) {
    argScreen.setXY(worldX, worldY);
    viewportTransform.getWorldToScreen(argScreen, argScreen);
  }

  /**
   * takes the world coordinate (argWorld) and returns the screen coordinates.
   * 
   * @param argWorld
   */
  Vec2 getWorldToScreen(Vec2 argWorld) {
    Vec2 screen = new Vec2.zero();
    viewportTransform.getWorldToScreen(argWorld, screen);
    return screen;
  }

  /**
   * Takes the world coordinates and returns the screen coordinates.
   * 
   * @param worldX
   * @param worldY
   */
  Vec2 getWorldToScreenXY(double worldX, double worldY) {
    Vec2 argScreen = new Vec2(worldX, worldY);
    viewportTransform.getWorldToScreen(argScreen, argScreen);
    return argScreen;
  }

  /**
   * takes the screen coordinates and puts the corresponding world coordinates in argWorld.
   * 
   * @param screenX
   * @param screenY
   * @param argWorld
   */
  void getScreenToWorldToOutXY(double screenX, double screenY, Vec2 argWorld) {
    argWorld.setXY(screenX, screenY);
    viewportTransform.getScreenToWorld(argWorld, argWorld);
  }

  /**
   * takes the screen coordinates (argScreen) and returns the world coordinates
   * 
   * @param argScreen
   */
  Vec2 getScreenToWorld(Vec2 argScreen) {
    Vec2 world = new Vec2.zero();
    viewportTransform.getScreenToWorld(argScreen, world);
    return world;
  }

  /**
   * takes the screen coordinates and returns the world coordinates.
   * 
   * @param screenX
   * @param screenY
   */
  Vec2 getScreenToWorldXY(double screenX, double screenY) {
    Vec2 screen = new Vec2(screenX, screenY);
    viewportTransform.getScreenToWorld(screen, screen);
    return screen;
  }
}
