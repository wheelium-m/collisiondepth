CC=$(shell (command -v clang++ >/dev/null && echo "clang++" || echo "g++"))
CFLAGS += -Iinclude $(shell pkg-config --cflags bullet)\
 $(shell pkg-config --cflags sdl)\
 $(shell pkg-config --cflags SDL_gfx) -O3 -Wall 

LIBS += $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)\
 $(shell pkf-config --libs tinyxml)

# Same as LIBS, but without the tinyxml dependency.
MACLIBS = $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)

OBJS = SDLBackend.o Model.o HeatPalette.o DepthMap.o StlFile.o\
 CollisionChecker.o PoseParser.o

all:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -lglut -lGLEW -lGL -g -ltinyxml

# Remove the DAE constant definition from CFLAGS
mac: CFLAGS:=$(subst -DDAE,,$(CFLAGS))

mac:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${MACLIBS} -framework OpenGL -framework GLUT

$(OBJS):	%.o: src/%.cpp $(wildcard include/*.h)
	${CC} ${CFLAGS} -c $< -o $@

.PHONY:	clean etags all

etags:
	find . -name '*.cpp' -or -name '*.h' | xargs etags

clean:
	rm -f *.o a.out
