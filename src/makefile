CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`
PYTHONCONFIG = /usr/include/python2.7/

relocalize_example: relocalize_example.o relocalize.o
	g++ relocalize_example.o relocalize.o -o relocalize_example $(CFLAGS) $(LIBS) -I $(PYTHONCONFIG) -I/usr/local/include/boost/python -lboost_python -lboost_system -std=c++11 -std=c++11

relocalize_example.o: relocalize_example.cpp
	g++ -c relocalize_example.cpp $(CFLAGS) $(LIBS) -I $(PYTHONCONFIG) -lboost_python -lboost_system -llibpython -std=c++11
 -lboost_python32 -lboost_system -std=c++11

relocalize.o: relocalize.cpp relocalize.h
	g++ -c relocalize.cpp $(CFLAGS) $(LIBS) -I $(PYTHONCONFIG)  -lboost_python -lboost_system -llibpython -std=c++11
 -lboost_python32 -lboost_system -std=c++11

clean:
	rm *.o relocalize_example relocalize
