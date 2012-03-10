/* Draws system status information. */
#ifndef RENDERER_HPP_
#define RENDERER_HPP_

#include "wheelchair_ros/Occupancy3D.h"

void checkGLError(const char* file, int line);

class Renderer;
class RendererView {
protected:
  Renderer &m_renderer;

public:
  RendererView(Renderer &r) : m_renderer(r) {}
  virtual void render() = 0;
};

class Renderer {
private:
  double m_fov;
  int m_width;
  int m_height;
  double m_azimuth;  // angle from the 'horizontal' plane
  double m_orient;   // angle about the vertical axis.
  double m_distance; // distance of the camera from the origin.

  int m_cube_list;

  RendererView *m_view;

public:
  Renderer(int width, int height);

  double getAzimuth() { return m_azimuth; }
  void setAzimuth(double a) { m_azimuth = a; }
  double getOrientation() { return m_orient; }
  void setOrientation(double o) { m_orient = o; }
  double getDistance() { return m_distance; }
  void setDistance(double d) { m_distance = d; }
  void setViewport(int w, int h);
  void setView(RendererView *view) { m_view = view; }

  void render();

  void drawCube(double x, double y, double z);

private:
  void createCubeDisplayList();
};

class Map3DView : public RendererView {
private:
  bool m_haveMap;
  wheelchair_ros::Occupancy3D::ConstPtr m_map;

public:
  Map3DView(Renderer &r) : RendererView(r), m_haveMap(false) {}

  virtual void render();
  void setMap(const wheelchair_ros::Occupancy3D::ConstPtr &msg);

private:
  void drawBounds(double x, double y, double z);
};

#endif //RENDERER_HPP_

