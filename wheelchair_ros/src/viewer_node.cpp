/* Uses GLUT to display the state of the system */

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ros/ros.h>
#include <iostream>
#include <memory>

#include <nav_msgs/OccupancyGrid.h>
#include "wheelchair_ros/PredictedPath.h"
#include "wheelchair_ros/Occupancy3D.h"
#include "wheelchair_ros/renderer.hpp"
#include "wheelchair_ros/geometry.hpp"

using namespace std;
using namespace wheelchair_ros;
using nav_msgs::OccupancyGrid;

const int TICK_MS = 33;
const int WIDTH = 600;
const int HEIGHT = 400;

static auto_ptr<Renderer> renderer (NULL);
static auto_ptr<Map3DView> map3View (NULL);
static auto_ptr<CollisionView> colView (NULL);
static auto_ptr<ros::NodeHandle> node (NULL);

void map3Callback(const Occupancy3D::ConstPtr &msg) {
  map3View->setMap(msg);
  glutPostRedisplay();
}

void map2Callback(const OccupancyGrid::ConstPtr &msg) {
  colView->setMap(msg);
  glutPostRedisplay();
}

void pathCallback(const PredictedPath::ConstPtr &msg) {
  map3View->setPath(msg);
  colView->setPath(msg);
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

void toggleDriveEnabled();
void keyHandler(unsigned char key, int x, int y) {
  switch(key) {
    case ' ':  // Reset camera position
      renderer->reset();
      break;
    case '1': // 3D map view
      renderer->setView(map3View.get());
      break;
    case '2': // Occupancy view
      renderer->setView(colView.get());
      break;
    case 'd': 
      toggleDriveEnabled();
      break;
      
    default:
      return;
  }
  glutPostRedisplay();
}

/* Mouse event handlers */
void buttonHandler(int, int, int, int);
void moveHandler(int, int);

int main(int argc, char *argv[]) {
  glutInit(&argc, argv);

  /* ROS init */
  ros::init(argc, argv, "viewer_node");
  node = auto_ptr<ros::NodeHandle> (new ros::NodeHandle);
  ros::Subscriber sub2 = node->subscribe<OccupancyGrid>(
      "map2d", 1, map2Callback);
  ros::Subscriber sub3 = node->subscribe<Occupancy3D>(
      "map3d", 1, map3Callback);
  ros::Subscriber pathSub = node->subscribe<PredictedPath>(
      "predicted_path", 1, pathCallback);

  /* GLUT init */
  glutInitWindowPosition(0, 0);
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Wheelchar 482");

  renderer = auto_ptr<Renderer> (new Renderer(WIDTH, HEIGHT));
  map3View = auto_ptr<Map3DView> (new Map3DView (*renderer));
  colView = auto_ptr<CollisionView> (new CollisionView (*renderer));
  renderer->setView(map3View.get());

  bool driveEnabled;
  node->param<bool>("drive_enabled", driveEnabled, true);
  renderer->setDriveEnabled(driveEnabled);

  glutDisplayFunc(render);
  glutTimerFunc(TICK_MS, tick, 0);
  glutReshapeFunc(onResize);
  glutMouseFunc(buttonHandler);
  glutMotionFunc(moveHandler);
  glutKeyboardFunc(keyHandler);
  glutMainLoop();
  return 0;
}

void toggleDriveEnabled() {
  bool cur;
  node->param<bool>("drive_enabled", cur, true);
  node->setParam("drive_enabled", !cur);
  renderer->setDriveEnabled(!cur);
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
  if (mouseState & RIGHT_DOWN) {
    Quaternion rot (M_PI*dy/180.0, Vector3D(1, 0, 0));
    rot = rot * Quaternion(M_PI*dx/180.0, Vector3D(0, 1, 0));
    renderer->setRotation(rot * renderer->getRotation());
  }
  if (mouseState & LEFT_DOWN) {
    if (glutGetModifiers() & GLUT_ACTIVE_SHIFT) {
      renderer->setTranslation(renderer->getTranslation() - Vector3D(0,0,dy));
    } else {
      renderer->setTranslation(renderer->getTranslation() + Vector3D(0.1*dx,-0.1*dy,0));
    }
  }

  lastMouseX = x;
  lastMouseY = y;
}

