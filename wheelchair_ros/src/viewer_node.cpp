/* Uses GLUT to display the state of the system */

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ros/ros.h>
#include <iostream>
#include <memory>

#include "wheelchair_ros/Occupancy3D.h"
#include "wheelchair_ros/renderer.hpp"

using namespace std;
using namespace wheelchair_ros;

const int TICK_MS = 33;
const int WIDTH = 600;
const int HEIGHT = 400;

static Renderer *renderer = NULL;
static Map3DView  *map3View = NULL;

void occupancyCallback(const Occupancy3D::ConstPtr &msg) {
  map3View->setMap(msg);
  glutPostRedisplay();
}

void render() {
  renderer->render();
}

void onResize(int width, int height) {
  renderer->setViewport(width, height);
  glutPostRedisplay();
}

/* Called at 20Hz.  Does ROS work, among other things. */
void tick(int value) {
  glutTimerFunc(TICK_MS, tick, 0);
  if (!ros::ok()) {
    exit(0);
  }
  ros::spinOnce();
  glutPostRedisplay();
}

/* Mouse motion state */
enum MouseStates {
  LEFT_DOWN = 1,
  RIGHT_DOWN = 2
};
static unsigned mouseState = 0;
static int lastMouseX = 0;
static int lastMouseY = 0;

void buttonHandler(int button, int state, int x, int y) {
  lastMouseX = x;
  lastMouseY = y;
  if (state == GLUT_DOWN) {
    switch (button) {
      case GLUT_LEFT_BUTTON:
        mouseState |= LEFT_DOWN;
        break;
      case GLUT_RIGHT_BUTTON:
        mouseState |= RIGHT_DOWN;
        break;
      default:
        break;
    }
  } else {
    switch (button) {
      case GLUT_LEFT_BUTTON:
        mouseState &= ~LEFT_DOWN;
        break;
      case GLUT_RIGHT_BUTTON:
        mouseState &= ~RIGHT_DOWN;
        break;
      default:
        break;
    }
  }
}

void moveHandler(int x, int y) {
  int dx = x - lastMouseX;
  int dy = y - lastMouseY;
  if (mouseState & LEFT_DOWN) {
    renderer->setOrientation(renderer->getOrientation() + dx);
    renderer->setAzimuth(renderer->getAzimuth() + dy);
  }
  if (mouseState & RIGHT_DOWN) {
    renderer->setDistance(renderer->getDistance() - dy);
    glutPostRedisplay();
  }

  lastMouseX = x;
  lastMouseY = y;
}

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);

  /* ROS init */
  ros::init(argc, argv, "viewer_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<Occupancy3D>(
      "map3d", 1, occupancyCallback);

  /* GLUT init */
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Wheelchar 482");

  renderer = new Renderer(WIDTH, HEIGHT);
  map3View = new Map3DView (*renderer);
  renderer->setView(map3View);

  glutDisplayFunc(render);
  glutTimerFunc(TICK_MS, tick, 0);
  glutReshapeFunc(onResize);
  glutMouseFunc(buttonHandler);
  glutMotionFunc(moveHandler);
  glutMainLoop();
  return 0;
}

