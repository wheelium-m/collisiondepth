CC=$(shell (command -v clang++ >/dev/null && echo "clang++" || echo "g++"))
CFLAGS = -lglut -lGLEW -lGL
CFLAGS_MAC = -lGLEW -framework OpenGL -framework GLUT

all: opengl

opengl: floats.bin
	$(CC) -o opengl opengl.cpp $(CFLAGS)

mac:	floats.bin opengl.cpp
	$(CC) -o opengl opengl.cpp $(CFLAGS_MAC)

floats.bin: makeFloats
	./makeFloats

makeFloats: makeFloats.cpp
	rm -f floats.bin
	$(CC) -o makeFloats makeFloats.cpp

.PHONY:	clean

clean:
	rm -f floats.bin *~ makeFloats opengl
