# Name of the executable
EXEC = kin_cpp

# Compiler and flags
CXX = clang++
CXXFLAGS = -Wall -Wextra -std=c++17 -I./third_party/blaze -I./third_party/gmm/gmm -I./third_party/scilab/include -I/opt/homebrew/opt/lapack/include

# LDFLAGS = -L./third_party/scilab/lib -lscilab

# LDFLAGS = -L/opt/homebrew/opt/lapack/lib
LAPACK_LIBS = -L/opt/homebrew/opt/lapack/lib -llapack -lblas

# Find all .cpp files recursively
# SRC = $(shell find . -path ./third_party -prune -o -name '*.cpp')
SRC = $(shell find . -path ./third_party -prune -o -name '*.cpp' -print)

# Generate object files in the same directories as the source files
OBJ = $(SRC:.cpp=.o)

# Default target
all: $(EXEC)

# Link object files into the final executable
$(EXEC): $(OBJ)
    $(CXX) $(CXXFLAGS) -o $(EXEC) $(OBJ) $(LAPACK_LIBS) $@ $^

# Compile each .cpp file into a .o file
# %.o: %.cpp
#   @mkdir -p $(dir $@)
#   $(CXX) $(CXXFLAGS) -c $< -o $@
%.o: %.cpp
    $(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up the build
clean:
    rm -f $(OBJ) $(EXEC)

# Phony targets
.PHONY: all clean


