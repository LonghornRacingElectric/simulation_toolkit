# Name of the executable
TARGET = kin_model

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -I./third_party/blaze -I./third_party/gmm/gmm -I./third_party/scilab/include
LDFLAGS = -L./third_party/scilab/lib -lscilab

# Find all .cpp files recursively
SRC = $(shell find . -name '*.cpp')

# Generate object files in the same directories as the source files
OBJ = $(SRC:.cpp=.o)

# Default target
all: $(TARGET)

# Link object files into the final executable
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

# Compile each .cpp file into a .o file
%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up the build
clean:
	rm -f $(OBJ) $(TARGET)

# Phony targets
.PHONY: all clean
