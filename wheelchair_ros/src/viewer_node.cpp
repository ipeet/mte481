/* Uses GLUT to display the state of the system */

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>

using namespace std;
using namespace nav_msgs;

const int TICK_MS = 33;

int width = 600;
int height = 400;
double rotation = 0.0;

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

void setupRender() {
  glShadeModel(GL_SMOOTH);
  glClearColor(0, 0, 0, 0);
  glEnable(GL_DEPTH_TEST);
  CHECK_GL();
}

void render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glEnable(GL_NORMALIZE);
  glEnable(GL_DEPTH_TEST);

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glViewport(0, 0, width, height);
  gluPerspective(50.0, double(width)/height, 0.1, 1000.0);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslated(0, 0, -2);
  glRotated(rotation, 0, 1, 0);
  glBegin(GL_TRIANGLES);
  glVertex3f(-0.5,-0.5,0.0);
  glVertex3f(0.5,0.0,0.0);
  glVertex3f(0.0,0.5,0.0);
  glEnd();

  glutSwapBuffers();
  CHECK_GL();
}

void onResize(int w, int h) {
  width = w;
  height = h;
  glutPostRedisplay();
}

void occupancyCallback(const OccupancyGrid::ConstPtr &msg) {
}

/* Called at 20Hz.  Does ROS work, among other things. */
void tick(int value) {
  glutTimerFunc(TICK_MS, tick, 0);
  if (!ros::ok()) {
    exit(0);
  }
  ros::spinOnce();
  rotation += 5.0;
  glutPostRedisplay();
}

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);

  /* ROS init */
  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<OccupancyGrid>(
      "map", 1, occupancyCallback);

  /* GLUT init */
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(width, height);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Wheelchar 482");
  glutDisplayFunc(render);
  glutTimerFunc(TICK_MS, tick, 0);
  glutReshapeFunc(onResize);
  setupRender();
  glutMainLoop();
  return 0;
}

