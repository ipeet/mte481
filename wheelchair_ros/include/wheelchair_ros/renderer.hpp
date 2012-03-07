/* Draws system status information. */
#ifndef RENDERER_HPP_
#define RENDERER_HPP_

class Renderer {
private:
  double m_rotation;
  double m_fov;
  int m_width;
  int m_height;

  int m_cube_list;

public:
  Renderer(int width, int height);

  double getRotation() { return m_rotation; }
  void setRotation(double rot) { m_rotation = rot; }
  void setViewport(int w, int h);

  void drawCube(double x, double y, double z);

  void render();

private:
  static void checkGLError(const char* file, int line);
  void createCubeDisplayList();
};

#endif //RENDERER_HPP_

