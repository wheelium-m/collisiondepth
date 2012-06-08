#include <OpenGL/gl.h>
#include <GLUT/glut.h>


void render() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  float ms = getDeltaTimeMicroseconds();

  glutSwapBuffers();
}

void reshape(GLsizei w, GLsizei h) {
  glViewport(0,0,w,h);
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  glOrtho(-50,50,-50,50,-1,1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}

void mouse(int button, int state, int x, int y) {
  exit();
}

void init() {
  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB);
  glutInitWindowSize(512,512);
  glutInitWindowPosition(100,100);
  glutCreateWindow("Depth Collision");
  glClearColor(0,0,0,0);

  glutDisplayFunc(render);
  glutReshapeFunc(reshape);
  glutMouseFunc(mouse);
  glutMainLoop();
}

