CC=$(shell (command -v clang++ >/dev/null && echo "clang++" || echo "g++"))
CFLAGS += -Iinclude $(shell pkg-config --cflags bullet) $(shell pkg-config --cflags sdl)\
 $(shell pkg-config --cflags SDL_gfx) -O3
LIBS += $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)
OBJS = SDLBackend.o Model.o MapMaker.o

all:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -lglut -lGLEW -lGL  -I/usr/include/ni -lOpenNI -g

mac:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -framework OpenGL -framework GLUT -I/usr/include/ni

$(OBJS):	%.o: src/%.cpp
	${CC} ${CFLAGS} -c $< -o $@ -I/usr/include/ni -lOpenNI

.PHONY:	clean

clean:
	rm -f *.o a.out