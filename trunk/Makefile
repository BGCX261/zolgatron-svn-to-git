CXX = g++ -m32
CXXFLAGS = -g -Wall -DdNODEBUG -I./include -I./include/littledog -I/. -I./simulator -D_THREAD_SAFE

LDFLAGS = -L/usr/lib/ -L./lib /usr/lib/libX11.so.6 /usr/lib/libGL.so.1 /usr/lib/libGLU.so.1 -L /afs/ir/class/cs221/LittleDogLib/ -lm -lode -ldrawstuff -lpthread ./lib/liblittledog.a 

OBJECTS_SIM = simulator/SDLittleDog.o simulator/SDSimulator.o simulator/SDTerrain.o simulator/SDUtils.o simulator/SDConstants.o simulator/SDKinematics.o SDSlowWalk.o Classifier.o 

OBJECTS = $(OBJECTS_SIM) aStar.o main.o amar_astar.o

all: default
default: depend main astar3

$(OBJECTS): %.o : %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

main: $(OBJECTS)
	$(CXX) -o main $(OBJECTS) $(LDFLAGS)

clean:
	rm $(OBJECTS) main

depend:
	$(CXX) $(CXXFLAGS) -MM *.cpp> Makefile.dep

ob: ob.cpp
	$(CXX) $(CXXFLAGS) -o ob ob.cpp $(OBJECTS_SIM) $(LDFLAGS)

astar3: astar3.cc gpath.cc
	$(CXX) $(CXXFLAGS) -o astar3 astar3.cc gpath.cc $(OBJECTS_SIM) $(LDFLAGS)

include Makefile.dep
