/* Draws system status information. */
#ifndef RENDERER_HPP_
#define RENDERER_HPP_

#include "wheelchair_ros/Occupancy3D.h"

class RendererView {
private:
  Renderer &m_renderer;

public:
  RendererView(Renderer &r) : m_renderer(r) {}
  virtual void render() = 0;
};

class Renderer {
private:
  double m_rotation;
  double m_fov;
  int m_width;
  int m_height;

  int m_cube_list;

  bool m_haveMap;
  wheelchair_ros::Occupancy3D::ConstPtr m_map;

public:
  Renderer(int width, int height, bv);

  double getRotation() { return m_rotation; }
  void setRotation(double rot) { m_rotation = rot; }
  void setViewport(int w, int h);

  void setMap(const wheelchair_ros::Occupancy3D::ConstPtr &msg);

  void render();

private:
  static void checkGLError(const char* file, int line);
  void createCubeDisplayList();
  void drawCube(double x, double y, double z);
  void drawBounds(double x, double y, double z);
  void drawMap();

  friend class RendererView;
};

#endif //RENDERER_HPP_

