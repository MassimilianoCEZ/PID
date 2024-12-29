# Compiler and standard
CXX = g++ -std=c++17

# Compiler flags
CXXFLAGS =
CXXFLAGS += -O0 -g        # No optimization, include debugging symbols
CXXFLAGS += -Wall         # Enable warnings
CXXFLAGS += -Wextra       # Enable extra warnings

# Output executable
EXE = pid.exe

# Default target
all: ${EXE}
	./${EXE}

# Valgrind target
valgrind: ${EXE}
	valgrind ./${EXE}

# Object file rules
pid-test.o: pid-test.cpp pid.h
	${CXX} ${CXXFLAGS} -c $< -o $@

pid.o: pid.h pid.cpp
	${CXX} ${CXXFLAGS} -c $< -o $@


# Link all object files into the executable
${EXE}: pid-test.o pid.o
	${CXX} ${CXXFLAGS} $^ -o $@

# Clean up build files
clean:
	rm -f *.o *.exe
