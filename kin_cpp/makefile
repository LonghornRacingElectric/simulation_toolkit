# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -I./third_party/blaze/ -I./third_party/gmm/gmm -I./third_party/scilab/include

# Find all .cpp files recursively
SRCS = $(shell find . -path ./third_party -prune -o -name '*.cpp')
OBJS = $(SRCS:.cpp=.o)
OBJS_FILTERED = $(shell find . -name '*.o')

# Executable name
EXEC = kin_model

# Generate object files in the same directories as the source files

# Default target
all: $(EXEC)

# Link object files into the final executable
$(EXEC): $(OBJS_FILTERED)
	$(CXX) $(CXXFLAGS) $(OBJS_FILTERED) -o $(EXEC)

# Compile each .cpp file into a .o file
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up the build
clean:
	rm -f $(OBJ) $(EXEC)

# Phony targets
.PHONY: all clean