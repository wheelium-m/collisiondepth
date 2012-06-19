CC = -Iinclude -I/usr/local/include/bullet   -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL   -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL   -O3 -L/usr/local/lib -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath   -lSDL   -lSDL_gfx -lSDL   -lglut -lGLEW -lGL

all:
	g++ -o spin spin.cpp $(CC)
