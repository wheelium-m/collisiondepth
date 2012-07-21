CC=$(shell (command -v clang++ >/dev/null && echo "clang++" || echo "g++"))
CFLAGS += -Iinclude $(shell pkg-config --cflags bullet)\
 $(shell pkg-config --cflags sdl)\
 $(shell pkg-config --cflags SDL_gfx) -O3 -Wall


LIBS += $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)\
 $(shell pkf-config --libs tinyxml)

OBJS = SDLBackend.o Model.o HeatPalette.o DepthMap.o StlFile.o

all:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -lglut -lGLEW -lGL -g -ltinyxml

mac:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -framework OpenGL -framework GLUT

$(OBJS):	%.o: src/%.cpp
	${CC} ${CFLAGS} -c $< -o $@

.PHONY:	clean etags

etags:
	find . -name '*.cpp' -or -name '*.h' | xargs etags

clean:
	rm -f *.o a.out