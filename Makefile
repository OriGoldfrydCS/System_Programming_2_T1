# Email: origoldbsc@gmail.com

# General macros
CXX = clang++
CXXFLAGS = -std=c++11 -Werror -Wsign-conversion
VALGRIND_FLAGS = -v --leak-check=full --show-leak-kinds=all --error-exitcode=99

# Macros for source files and headers files
SOURCES = Graph.cpp Algorithms.cpp
HEADERS = Graph.hpp Algorithms.hpp
DEMO_SRC = Demo.cpp
TEST_SRC = Test.cpp
TEST_COUNTER_SRC = TestCounter.cpp

# Macros for object and headers files
OBJECTS = Graph.o Algorithms.o

# Main target: Build and run the demo 
run: demo
	./demo

# Build the demo exe file from object files
demo: Demo.o $(OBJECTS)
	$(CXX) $(CXXFLAGS) Demo.o $(OBJECTS) -o demo

# Build the test exe file that includes the tests
test: TestCounter.o Test.o $(OBJECTS)
	$(CXX) $(CXXFLAGS) TestCounter.o Test.o $(OBJECTS) -o test

# Run clang-tidy
tidy:
	clang-tidy $(SOURCES) -checks=bugprone-*,clang-analyzer-*,cppcoreguidelines-*,performance-*,portability-*,readability-*,-cppcoreguidelines-pro-bounds-pointer-arithmetic,-cppcoreguidelines-owning-memory --warnings-as-errors=-* --

# Run valgrind 
valgrind: demo test
	valgrind --tool=memcheck $(VALGRIND_FLAGS) ./demo 2>&1 | { egrep "lost| at " || true; }
	valgrind --tool=memcheck $(VALGRIND_FLAGS) ./test 2>&1 | { egrep "lost| at " || true; }

# Rule to compile Graph object file
Graph.o: Graph.cpp Graph.hpp
	$(CXX) $(CXXFLAGS) -c Graph.cpp -o Graph.o

# Rule to compile Algorithms object file
Algorithms.o: Algorithms.cpp Algorithms.hpp
	$(CXX) $(CXXFLAGS) -c Algorithms.cpp -o Algorithms.o

# Rule to compile Demo object file
Demo.o: Demo.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c Demo.cpp -o Demo.o

# Rule to compile Test object file
Test.o: Test.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c Test.cpp -o Test.o

# Rule to compile TestCounter object file
TestCounter.o: TestCounter.cpp $(HEADERS)
	$(CXX) $(CXXFLAGS) -c TestCounter.cpp -o TestCounter.o

# Clean up command to remove all compiled files
clean:
	rm -f *.o demo test
