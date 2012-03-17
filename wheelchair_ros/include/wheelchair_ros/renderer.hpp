/* Draws system status information. */
#ifndef RENDERER_HPP_
#define RENDERER_HPP_

#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/Occupancy3D.h"
#include "wheelchair_ros/PredictedPath.h"
#include "wheelchair_ros/geometry.hpp"

void checkGLError(const char* file, int line);

class Polygon;

class Renderer;
class RendererView {
protected:
  Renderer &m_renderer;
  Quaternion m_rotation;
  Vector3D m_translation;

  const static Quaternion DEFAULT_ROTATION;
  const static Vector3D DEFAULT_TRANSLATION;

public:
  RendererView(Renderer &r) : m_renderer(r) {reset();}
  virtual void render() = 0;
  virtual void reset();  // Resets co-ordinate transforms

  friend class Renderer;
};

class Renderer {
private:
  double m_fov;
  int m_width;
  int m_height;

  int m_cube_list;

  RendererView *m_view;

public:
  Renderer(int width, int height);

  Quaternion getRotation() const;
  void setRotation(const Quaternion &r);
  Vector3D getTranslation() const;
  void setTranslation(const Vector3D &v);
  void reset(); // Resets to default view configuration

  void setViewport(int w, int h);
  void setView(RendererView *view) { m_view = view; }

  void render();

  void drawCube(double x, double y, double z);
  void drawQuad(double x, double y);
  void drawPoly(const Polygon &poly);

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

class CollisionView : public RendererView { 
private:
  nav_msgs::OccupancyGrid::ConstPtr m_map;
  bool m_haveMap;
  wheelchair_ros::PredictedPath::ConstPtr m_path;
  bool m_havePath;

public:
  CollisionView(Renderer &r) : 
    RendererView(r), 
    m_haveMap(false),
    m_havePath(false)
  { reset(); }

  virtual void render();
  virtual void reset();
  void setMap(const nav_msgs::OccupancyGrid::ConstPtr &msg);
  void setPath(const wheelchair_ros::PredictedPath::ConstPtr &msg);

protected:
  void renderMap();
  void renderPath();
};

#endif //RENDERER_HPP_
