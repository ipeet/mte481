#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>

#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/renderer.hpp"
#include "wheelchair_ros/Occupancy3D.h"

using namespace std;
using nav_msgs::OccupancyGrid;
using nav_msgs::Path;

void checkGLError(const char* file, int line) {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    if (file) cerr << file << ":";
    if (line) cerr << line << ":";
    if (file || line) cerr << " ";
    cerr << gluErrorString(err) << endl;
  }
}
#define CHECK_GL() checkGLError(__FILE__, __LINE__)

const double Renderer::DEFAULT_AZIMUTH = 30.0;
const double Renderer::DEFAULT_ORIENT = 0.0;
const double Renderer::DEFAULT_DISTANCE = 100.0;

Renderer::Renderer(int width, int height) :
  m_fov(60.0),
  m_width(width),
  m_height(height),
  m_azimuth(DEFAULT_AZIMUTH),
  m_orient(DEFAULT_ORIENT),
  m_distance(DEFAULT_DISTANCE),
  m_cube_list(0),
  m_view(NULL)
{
  glShadeModel(GL_SMOOTH);
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  CHECK_GL();
}

void Renderer::render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);
  glCullFace(GL_BACK);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, m_width, m_height);
  gluPerspective(m_fov, double(m_width)/m_height, 0.1, 1000.0);
  CHECK_GL();

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  /* Lighting */
  glEnable(GL_LIGHTING);
  GLfloat pos[] = {0, 0, 0, 1.0};
  glLightfv(GL_LIGHT1, GL_POSITION, pos);
  GLfloat light[] = {0.7, 0.7, 0.7};
  GLfloat dark[] = {0, 0, 0};
  glLightfv(GL_LIGHT1, GL_AMBIENT, dark);
  glLightfv(GL_LIGHT1, GL_DIFFUSE, light);
  glLightfv(GL_LIGHT1, GL_SPECULAR, light);
  glEnable(GL_LIGHT1);

  /* Transform to world co-ordinates */
  glPushMatrix();
  glTranslated(0, 0, -m_distance);
  glRotated(m_azimuth, 1, 0, 0);
  glRotated(m_orient, 0, 1, 0);

  if (!m_view) {
    glTranslated(-0.5, 0, -0.5);
    glScalef(10.0, 10.0, 10.0);
    drawCube(0, 0, 0);
    drawCube(1, 0, 0);
    drawCube(1, 0, 1);
    drawCube(0, 1, 0);
    drawCube(0, 1, 1);
    drawCube(1, 1, 1); 
  } else {
    m_view->render();
  }

  glPopMatrix();
  CHECK_GL();

  glutSwapBuffers();
}

void Renderer::setViewport(int w, int h) {
  m_width = w;
  m_height = h;
}

void Renderer::reset() {
  m_azimuth = DEFAULT_AZIMUTH;
  m_orient = DEFAULT_ORIENT;
  m_distance = DEFAULT_DISTANCE;
}

void Renderer::drawCube(double x, double y, double z) {
  if (!m_cube_list) {
    createCubeDisplayList();
  }

  glPushMatrix();
  glTranslated(x, y, z);
  glScaled(0.9, 0.9, 0.9);

  glCallList(m_cube_list);
  glPopMatrix();
  CHECK_GL();
}

void Renderer::drawQuad(double x, double y) {
  glPushMatrix();
  glTranslated(x, y, 0);
  glBegin(GL_QUADS);
  glNormal3d(0, 0, 1);
  glVertex3d(-0.5, -0.5, 0);
  glVertex3d(0.5, -0.5, 0);
  glVertex3d(0.5, 0.5, 0);
  glVertex3d(-0.5, 0.5, 0);
  glEnd();
  glPopMatrix();
  CHECK_GL();
}

void Renderer::createCubeDisplayList() {
  m_cube_list = glGenLists(1);
  if (! m_cube_list) {
    cerr << "Failed to allocate display list." << endl;
    cerr << "GL: " << gluErrorString(glGetError()) << endl;
    abort();
  }

  glNewList(m_cube_list, GL_COMPILE);
  glBegin(GL_QUADS);
  /* xy faces */
  glNormal3d(0, 0, 1);
  glVertex3d(-0.5, -0.5, 0.5);
  glVertex3d(0.5, -0.5, 0.5);
  glVertex3d(0.5, 0.5, 0.5);
  glVertex3d(-0.5, 0.5, 0.5);

  glNormal3d(0, 0, -1);
  glVertex3d(0.5, -0.5, -0.5);
  glVertex3d(-0.5, -0.5, -0.5);
  glVertex3d(-0.5, 0.5, -0.5);
  glVertex3d(0.5, 0.5, -0.5);

  /* yz faces */
  glNormal3d(1, 0, 0);
  glVertex3d(0.5, -0.5, -0.5);
  glVertex3d(0.5, 0.5, -0.5);
  glVertex3d(0.5, 0.5, 0.5);
  glVertex3d(0.5, -0.5, 0.5);

  glNormal3d(-1, 0, 0);
  glVertex3d(-0.5, -0.5, 0.5);
  glVertex3d(-0.5, 0.5, 0.5);
  glVertex3d(-0.5, 0.5, -0.5);
  glVertex3d(-0.5, -0.5, -0.5);

  /* xz faces */
  glNormal3d(0, 1, 0);
  glVertex3d(0.5, 0.5, -0.5);
  glVertex3d(-0.5, 0.5, -0.5);
  glVertex3d(-0.5, 0.5, 0.5);
  glVertex3d(0.5, 0.5, 0.5);

  glNormal3d(0, -1, 0);
  glVertex3d(-0.5, -0.5, -0.5);
  glVertex3d(0.5, -0.5, -0.5);
  glVertex3d(0.5, -0.5, 0.5);
  glVertex3d(-0.5, -0.5, 0.5);

  glEnd();
  glEndList();
}

void Map3DView::setMap(const wheelchair_ros::Occupancy3D::ConstPtr &msg) {
  m_haveMap = true;
  m_map = msg;
}

void Map3DView::render() {
  glPushMatrix();

  /* convenience */
  int w, d, h;
  if (m_haveMap) {
    w = m_map->width;
    d = m_map->depth;
    h = m_map->height;
  } else {
    w = d = 40;
    h = 10;
  }

  // X offset to algin the centre the map's depth axis.
  glTranslated(-0.5*w, -0.5*h, 0.5*d);

  drawBounds(w, h, -d);

  if (!m_haveMap) {
    glPopMatrix();
    CHECK_GL();
    return;
  }

  /* Set cube material */
  GLfloat diff[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat spec[] = {1.0, 1.0, 1.0, 1.0};
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, diff);
  glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
  glMaterialf(GL_FRONT, GL_SHININESS, 20.0);
  for (int x=0; x < w; ++x) {
    for (int z=0; z < d; ++z) {
      for (int y=0; y < h; ++y) {
         int data = m_map->data[d*w*y + w*z + x];
         if (data) {
           m_renderer.drawCube(x, y, -z);
         }
      }
    }
  }

  glPopMatrix();
  CHECK_GL();
}

void Map3DView::drawBounds(double x, double y, double z) {
  GLfloat em[] = {0.0, 1.0, 0.0, 1.0};
  glMaterialfv(GL_FRONT, GL_EMISSION, em);

  /* Back wall */
  glBegin(GL_LINE_LOOP);
  glVertex3d(0,0,z);
  glVertex3d(x,0,z);
  glVertex3d(x,y,z);
  glVertex3d(0,y,z);
  glEnd();

  /* Left wall */
  glBegin(GL_LINE_LOOP);
  glVertex3d(0,0,0); 
  glVertex3d(0,0,z); 
  glVertex3d(0,y,z); 
  glVertex3d(0,y,0); 
  glEnd();

  /* Right Wall */
  glBegin(GL_LINE_LOOP);
  glVertex3d(x,0,0); 
  glVertex3d(x,0,z); 
  glVertex3d(x,y,z); 
  glVertex3d(x,y,0); 
  glEnd();

  /* Front wall(ish) */
  glBegin(GL_LINES);
  glVertex3d(0,0,0);
  glVertex3d(x,0,0);
  glEnd();

  GLfloat no_em[] = {0, 0, 0, 0};
  glMaterialfv(GL_FRONT, GL_EMISSION, no_em);
  CHECK_GL();
}

void CollisionView::setMap(const OccupancyGrid::ConstPtr &msg) {
  m_map = msg;
  m_haveMap = true;
}

void CollisionView::setPath(const Path::ConstPtr &msg) {
  m_path = msg;
  m_havePath = true;
  cerr << "Path!" << endl;
}

void CollisionView::render() {
  glPushMatrix();

  double w, h;
  if (m_haveMap) {
    w = m_map->info.width;
    h = m_map->info.height;
  } else {
    w = 40;
    h = 40;
  }

  /* Invert the default rotations.  The means that map will be viewed head-on
   * if view hasn't been modified, but will retain any delta */
  glRotated(-Renderer::DEFAULT_ORIENT, 0, 1, 0);
  glRotated(-Renderer::DEFAULT_AZIMUTH, 1, 0, 0);
  glTranslated(-0.5*w, -0.5*h, 0);

  /* Draw the bouding rectangle */
  GLfloat em[] = {0, 0, 0.8, 1};
  GLfloat blk[] = {0, 0, 0, 1};
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, blk);
  glMaterialfv(GL_FRONT, GL_SPECULAR, blk);
  glMaterialfv(GL_FRONT, GL_EMISSION, em);
  glBegin(GL_LINE_LOOP);
  glVertex3d(0, 0, 0);
  glVertex3d(w, 0, 0);
  glVertex3d(w, h, 0);
  glVertex3d(0, h, 0);
  glEnd();

  if (!m_haveMap) {
    glPopMatrix();
    CHECK_GL();
    return;
  }

  /* Draw the actual map */
  GLfloat diff[] = {1.0, 1.0, 1.0, 1.0};
  GLfloat spec[] = {1.0, 1.0, 1.0, 1.0};
  glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, diff);
  glMaterialfv(GL_FRONT, GL_SPECULAR, spec);
  glMaterialfv(GL_FRONT, GL_EMISSION, blk);
  glMaterialf(GL_FRONT, GL_SHININESS, 20.0);
  for (int i=0; i < w; ++i) {
    for (int j=0; j < h; ++j) {
      if (m_map->data[j*w + i]) {
        m_renderer.drawQuad(i, j);
      }
    }
  }

  glPopMatrix();
  CHECK_GL();
}

