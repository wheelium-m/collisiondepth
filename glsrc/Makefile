CC = g++
CFLAGS = -lglut -lGLEW -lGL

all: opengl

opengl: floats.bin
	$(CC) -o opengl opengl.cpp $(CFLAGS)

floats.bin: makeFloats
	./makeFloats

makeFloats: makeFloats.cpp
	rm -f floats.bin
	$(CC) -o makeFloats makeFloats.cpp

clean:
	rm -f floats.bin *~