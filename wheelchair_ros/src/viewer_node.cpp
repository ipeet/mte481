/* Uses GLUT to display the state of the system */

#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <iostream>
#include <memory>

#include "wheelchair_ros/renderer.hpp"

using namespace std;
using namespace nav_msgs;

const int TICK_MS = 33;
const int WIDTH = 600;
const int HEIGHT = 400;


Renderer& renderer() {
  static auto_ptr<Renderer> r (NULL);
  if (r.get() == NULL) {
    r = auto_ptr<Renderer> (new Renderer(WIDTH, HEIGHT));
  }
  return *r; 
}

void occupancyCallback(const OccupancyGrid::ConstPtr &msg) {

}

void render() {
  renderer().render();
}

void onResize(int width, int height) {
  renderer().setViewport(width, height);
  glutPostRedisplay();
}

/* Called at 20Hz.  Does ROS work, among other things. */
void tick(int value) {
  glutTimerFunc(TICK_MS, tick, 0);
  if (!ros::ok()) {
    exit(0);
  }
  ros::spinOnce();
  renderer().setRotation(renderer().getRotation() + 1.0);
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
  glutInitWindowSize(WIDTH, HEIGHT);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
  glutCreateWindow("Wheelchar 482");
  glutDisplayFunc(render);
  glutTimerFunc(TICK_MS, tick, 0);
  glutReshapeFunc(onResize);
  glutMainLoop();
  return 0;
}

