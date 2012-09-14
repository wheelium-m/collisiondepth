CC=$(shell (command -v clang++ >/dev/null && echo "clang++" || echo "g++"))
CFLAGS += -Iinclude $(shell pkg-config --cflags bullet)\
 $(shell pkg-config --cflags sdl)\
 $(shell pkg-config --cflags SDL_gfx) -O3 -Wall
# -DFISHEYE
# -DSPHERE_RADIUS=0.2 -DMODEL_SCALE=0.5

# --std=c++11 -stdlib=libc++

LIBS += $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)\
 $(shell pkg-config --libs tinyxml)

# Same as LIBS, but without the tinyxml dependency.
MACLIBS = $(shell pkg-config --libs bullet) $(shell pkg-config --libs sdl)\
 $(shell pkg-config --libs SDL_gfx)

OBJS = SDLBackend.o Model.o HeatPalette.o DepthMap.o StlFile.o\
 CollisionChecker.o PoseParser.o YMCA.o Planner.o CollisionModel.o\
 PathHelper.o

all:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${LIBS} -lglut -lGL -g -ltinyxml

# Remove the DAE constant definition from CFLAGS
mac: CFLAGS:=$(subst -DDAE,,$(CFLAGS)) -DMAC

mac:	src/main.cpp ${OBJS}
	${CC} $^ ${CFLAGS} ${MACLIBS} -framework OpenGL -framework GLUT

$(OBJS):	%.o: src/%.cpp $(wildcard include/*.h)
	${CC} ${CFLAGS} -c $< -o $@

.PHONY:	clean etags all

etags:
	find . -name '*.cpp' -or -name '*.h' | xargs etags

clean:
	rm -f *.o a.out
