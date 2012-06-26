CC = -Iinclude -I/usr/local/include/bullet -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL -D_GNU_SOURCE=1 -D_REENTRANT -I/usr/include/SDL -O3 -L/usr/local/lib -lBulletSoftBody -lBulletDynamics -lBulletCollision -lLinearMath -lSDL -lSDL_gfx -lSDL -lglut -lGLEW -lGL

main: main.o Joint.o Model.o
	g++ -o main main.o Joint.o Model.o $(CC)

main.o: main.cpp
	g++ -o main.o main.cpp -c $(CC)

Joint.o: Joint.cpp
	g++ -o Joint.o Joint.cpp -c $(CC)

Model.o: Model.cpp
	g++ -o Model.o Model.cpp -c $(CC)

clean:
	rm -rf *.o *~ main