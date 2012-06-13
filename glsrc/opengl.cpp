#include <stdlib.h>
#include <GL/glew.h>
#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif
#include <math.h>
#include <stdio.h>
#include <iostream>
#include <fstream>

using namespace std;

/*
 * Global data used by our render callback:
 */

static float rotation;

static struct {
    GLuint vertex_buffer, element_buffer;
    GLuint texture;
    GLuint vertex_shader, fragment_shader, program;
    
    struct {
      GLint texture;
      GLint rotation;
    } uniforms;

    struct {
        GLint position;
    } attributes;

 
} g_resources;

/*
 * Functions for creating OpenGL objects:
 */

/*
void *getContents(const char *filename, GLint *length)
{
 
  FILE *f = fopen(filename, "r");
  void *buffer;

  if (!f) {
    fprintf(stderr, "Unable to open %s for reading\n", filename);
    return NULL;
  }

  fseek(f, 0, SEEK_END);
  *length = ftell(f);
  fseek(f, 0, SEEK_SET);

  buffer = malloc(*length+1);
  *length = fread(buffer, 1, *length, f);
  fclose(f);
  ((char*)buffer)[*length] = '\0';

  return buffer;
  }
*/
void * getContents(const char * filename, GLint * length, bool addnull){
  ifstream file(filename);
  int x = 0;
  if(addnull)
    x = 1;
  file.seekg(0, ios::end);
  *length = file.tellg();
  file.seekg(0, ios::beg);
  void * buffer;
  buffer = malloc(*length + x);
  file.read((char *)buffer, *length);
  if(addnull)
    ((char *)buffer)[*length] = '\0';
  file.close();
  return buffer;
  }

void * getContentsBinary(const char * filename, int * length){
  
  ifstream file("floats.bin");
  file.seekg(0, ios::end);
  *length = file.tellg();
  file.seekg(0, ios::beg);
  void * buffer;
  buffer = malloc(*length);
  file.read((char *)buffer, *length);
  file.close();
  


  return buffer;
  }


static GLuint make_buffer(
    GLenum target,
    const void *buffer_data,
    GLsizei buffer_size
) {
    GLuint buffer;
    glGenBuffers(1, &buffer);
    glBindBuffer(target, buffer);
    glBufferData(target, buffer_size, buffer_data, GL_STATIC_DRAW);
    return buffer;
}

static GLuint make_texture(const char *filename)
{
    int width=100, height=100;
    int length;
    void *buffer = getContentsBinary(filename, &length);
    GLuint texture;
    
    if (!buffer)
        return 0;

    glGenTextures(1, &texture);
    glBindTexture(GL_TEXTURE_2D, texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S,     GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T,     GL_CLAMP_TO_EDGE);
    glTexImage2D(
        GL_TEXTURE_2D, 0,           /* target, level */
        GL_RGB32F_ARB,                    /* internal format */
        width, height, 0,           /* width, height, border */
        GL_RGB, GL_FLOAT,   /* external format, type */
        buffer                      /* pixels */
    );
    
    
    free(buffer);
    return texture;
}

static void show_info_log(
    GLuint object,
    PFNGLGETSHADERIVPROC glGet__iv,
    PFNGLGETSHADERINFOLOGPROC glGet__InfoLog
)
{
    GLint log_length;
    char *log;

    glGet__iv(object, GL_INFO_LOG_LENGTH, &log_length);
    log = (char *)malloc(log_length);
    glGet__InfoLog(object, log_length, NULL, log);
    fprintf(stderr, "%s", log);
    free(log);
}

static GLuint make_shader(GLenum type, const char *filename)
{
    GLint length;
    GLchar *source = (GLchar *)getContents(filename, &length, false);
    GLuint shader;
    GLint shader_ok;

    if (!source)
        return 0;

    shader = glCreateShader(type);
    glShaderSource(shader, 1, (const GLchar**)&source, &length);
    free(source);
    glCompileShader(shader);

    glGetShaderiv(shader, GL_COMPILE_STATUS, &shader_ok);
    if (!shader_ok) {
        fprintf(stderr, "Failed to compile %s:\n", filename);
        show_info_log(shader, glGetShaderiv, glGetShaderInfoLog);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

static GLuint make_program(GLuint vertex_shader, GLuint fragment_shader)
{
    GLint program_ok;

    GLuint program = glCreateProgram();

    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);

    glGetProgramiv(program, GL_LINK_STATUS, &program_ok);
    if (!program_ok) {
        fprintf(stderr, "Failed to link shader program:\n");
        show_info_log(program, glGetProgramiv, glGetProgramInfoLog);
        glDeleteProgram(program);
        return 0;
    }
    return program;
}

/*
 * Data used to seed our vertex array and element array buffers:
 */
static const GLfloat g_vertex_buffer_data[] = { 
    0.0f, 0.0f,
     1.0f, 0.0f,
    0.0f,  1.0f
 
};
static const GLushort g_element_buffer_data[] = { 0, 1, 2 };

/*
 * Load and create all of our resources:
 */
static int make_resources(void)
{
    g_resources.vertex_buffer = make_buffer(
        GL_ARRAY_BUFFER,
        g_vertex_buffer_data,
        sizeof(g_vertex_buffer_data)
    );
    g_resources.element_buffer = make_buffer(
        GL_ELEMENT_ARRAY_BUFFER,
        g_element_buffer_data,
        sizeof(g_element_buffer_data)
    );

    g_resources.texture = make_texture("floats.bin");

    if (g_resources.texture == 0)
        return 0;

    g_resources.vertex_shader = make_shader(
        GL_VERTEX_SHADER,
        "vert_shader.glsl"
    );
    if (g_resources.vertex_shader == 0)
        return 0;

    g_resources.fragment_shader = make_shader(
        GL_FRAGMENT_SHADER,
        "frag_shader.glsl"
    );
    if (g_resources.fragment_shader == 0)
        return 0;

    g_resources.program = make_program(g_resources.vertex_shader, g_resources.fragment_shader);
    if (g_resources.program == 0)
        return 0;

    g_resources.uniforms.texture
        = glGetUniformLocation(g_resources.program, "texture");

    g_resources.uniforms.rotation
        = glGetUniformLocation(g_resources.program, "rotation");

    g_resources.attributes.position
        = glGetAttribLocation(g_resources.program, "position");

    return 1;
}

/*
 * GLUT callbacks:
 */
void idle(void){
  rotation+=3.1415926535/1000;
  if(rotation > 6.283852){
    rotation = 0.0;
  }
  usleep(100);
  glutPostRedisplay();
}

static void render(void)
{
  glClearColor(0.0f,0.0f,0.0f,0.0f);
  glClear(GL_COLOR_BUFFER_BIT);
    glUseProgram(g_resources.program);

 
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, g_resources.texture);
    glUniform1i(g_resources.uniforms.texture, 0);

    glUniform1f(g_resources.uniforms.rotation, rotation);

    glBindBuffer(GL_ARRAY_BUFFER, g_resources.vertex_buffer);
    glVertexAttribPointer(
        g_resources.attributes.position,  /* attribute */
        2,                                /* size */
        GL_FLOAT,                         /* type */
        GL_FALSE,                         /* normalized? */
        sizeof(GLfloat)*2,                /* stride */
        (void*)0                          /* array buffer offset */
    );
    glEnableVertexAttribArray(g_resources.attributes.position);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, g_resources.element_buffer);
    glDrawElements(
        GL_TRIANGLES,  /* mode */
        3,                  /* count */
        GL_UNSIGNED_SHORT,  /* type */
        (void*)0            /* element array buffer offset */
    );

    glDisableVertexAttribArray(g_resources.attributes.position);
    glutSwapBuffers();
}

/*
 * Entry point
 */
int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(400, 300);
    glutCreateWindow("Hello World");
    glutIdleFunc(&idle);
    glutDisplayFunc(&render);

    glewInit();
    if (!GLEW_VERSION_2_0) {
        fprintf(stderr, "OpenGL 2.0 not available\n");
        return 1;
    }

    if (!make_resources()) {
        fprintf(stderr, "Failed to load resources\n");
        return 1;
    }

    glutMainLoop();
    return 0;
}

