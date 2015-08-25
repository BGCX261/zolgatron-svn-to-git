CXX = g++ -m32
CXXFLAGS = -g -Wall -DdNODEBUG -I../include -I../include/littledog -I/. -I../simulator -D_THREAD_SAFE
LDFLAGS = -L/usr/lib/ -L ../lib /usr/lib/libX11.so.6 /usr/lib/libGL.so.1 /usr/lib/libGLU.so.1 -L /afs/ir/class/cs221/LittleDogLib/ -lm -lode -ldrawstuff

OBJECTS = tergen.o Terrain.o

default: depend main

$(OBJECTS): %.o : %.cpp
	$(CXX) $(CXXFLAGS) -o $@ -c $<

main: $(OBJECTS)
	$(CXX) -o tergen $(OBJECTS) $(LDFLAGS)

clean:
	rm $(OBJECTS) tergen

depend:
	$(CXX) $(CXXFLAGS) -MM *.cpp> Makefile.dep

include Makefile.dep
