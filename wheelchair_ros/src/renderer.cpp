#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <iostream>

#include "wheelchair_ros/renderer.hpp"
#include "wheelchair_ros/Occupancy3D.h"

using namespace std;

void Renderer::checkGLError(const char* file, int line) {
  GLenum err = glGetError();
  if (err != GL_NO_ERROR) {
    if (file) cerr << file << ":";
    if (line) cerr << line << ":";
    if (file || line) cerr << " ";
    cerr << gluErrorString(err) << endl;
  }
}
#define CHECK_GL() checkGLError(__FILE__, __LINE__)

Renderer::Renderer(int width, int height) :
  m_rotation(-1.0),
  m_fov(60.0),
  m_width(width),
  m_height(height),
  m_haveMap(false)
{
  glShadeModel(GL_SMOOTH);
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  CHECK_GL();

  createCubeDisplayList();
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
  glTranslated(0, 0, -40);
  glRotated(45, 1, 0, 0);
  glRotated(m_rotation, 0, 1, 0);

  if (!m_haveMap) {
    glTranslated(-0.5, 0, -0.5);
    drawCube(0, 0, 0);
    drawCube(1, 0, 0);
    drawCube(1, 0, 1);
    drawCube(0, 1, 0);
    drawCube(0, 1, 1);
    drawCube(1, 1, 1); 
  } else {
    drawMap();
  }

  glPopMatrix();
  CHECK_GL();

  glutSwapBuffers();
}

void Renderer::setViewport(int w, int h) {
  m_width = w;
  m_height = h;
}

void Renderer::setMap(const wheelchair_ros::Occupancy3D::ConstPtr &msg) {
  m_haveMap = true;
  m_map = msg;
}

void Renderer::drawMap() {
  glPushMatrix();

  /* convenience */
  int w = m_map->width;
  int d = m_map->depth;
  int h = m_map->height;

  // X offset to algin the centre the map's depth axis.
  glTranslatef(-0.5*w, 0, 0);

  for (int i=0; i < w; ++i) {
    for (int j=0; j < d; ++j) {
      for (int k=0; k < h; ++k) {
         int data = m_map->data[d*w*k + w*j + i];
         if (data) {
           drawCube(i, k, j);
         }
      }
    }
  }

  glPopMatrix();
  CHECK_GL();
}

void Renderer::drawCube(double x, double y, double z) {
  glPushMatrix();
  glTranslated(x, y, z);
  glScaled(0.9, 0.9, 0.9);
  glCallList(m_cube_list);
  glPopMatrix();
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

