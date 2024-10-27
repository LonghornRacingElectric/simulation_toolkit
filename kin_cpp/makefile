# # Compiler and flags
# CXX = g++
# CXXFLAGS = -Wall -Wextra -std=c++17 -Ithird_party/blaze

# # Directories
# SRC_DIRS = kernel primary_elements secondary_elements tertiary_elements assets
# BUILD_DIR = build

# # Find all .cpp and corresponding .o files
# SRC_FILES := $(shell find $(SRC_DIRS) -name '*.cpp')
# OBJ_FILES := $(patsubst %, $(BUILD_DIR)/%, $(SRC_FILES:.cpp=.o))

# # Target executable
# TARGET = program

# # Default target
# all: $(TARGET)

# # Build the program
# $(TARGET): $(OBJ_FILES)
# 	$(CXX) $(CXXFLAGS) -o $@ $^

# # Compile .cpp files into .o files
# $(BUILD_DIR)/%.o: %.cpp | create_dirs
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

# # Ensure build directories exist
# create_dirs:
# 	@mkdir -p $(patsubst %, $(BUILD_DIR)/%, $(SRC_DIRS))

# # Clean the build
# clean:
# 	rm -rf $(BUILD_DIR) $(TARGET)

# .PHONY: all clean create_dirs

# Compiler and flags
CXX = g++
CXXFLAGS = -Wall -Wextra -std=c++17 -Ithird_party/blaze -llapack -lblas

# Directories
SRC_DIRS = kernel primary_elements secondary_elements tertiary_elements assets
BUILD_DIR = build

# Find all .cpp and corresponding .o files
SRC_FILES := $(shell find $(SRC_DIRS) -name '*.cpp')
OBJ_FILES := $(patsubst %, $(BUILD_DIR)/%, $(SRC_FILES:.cpp=.o))

# Target executable
TARGET = program

# Default target
all: $(TARGET)

# Build the program
$(TARGET): $(OBJ_FILES)
	$(CXX) $(CXXFLAGS) -o $@ $^


# Compile .cpp files into .o files
$(BUILD_DIR)/%.o: %.cpp | create_dirs
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Ensure build directories exist
create_dirs:
	@mkdir -p $(patsubst %, $(BUILD_DIR)/%, $(SRC_DIRS))

# Clean the build
clean:
	rm -rf $(BUILD_DIR) $(TARGET)

.PHONY: all clean create_dirs
