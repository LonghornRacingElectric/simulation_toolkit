cmake_minimum_required(VERSION 3.0.0) 
project(OpenGL_Project VERSION 0.1.0)
cmake_policy(SET CMP0072 NEW)

find_package(OpenGL REQUIRED)

add_executable(OpenGL_Project 
    src/main.cpp 
    src/glad.h 
    src/glad.c
)

target_link_libraries(OpenGL_Project 
    glfw
    OpenGL::GL    
)
