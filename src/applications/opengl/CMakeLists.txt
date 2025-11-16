Cmake_minimum_required(VERSION 3.5.0)
project(opengl_testing VERSION 0.1.0)
cmake_policy(SET CMP0072 NEW)

find_package(OpenGL REQUIRED)

add_executable(opengl_testing 
    src/main.cpp 
    src/glad.h 
    src/glad.c 
    imgui/imconfig.h 
    imgui/imgui_demo.cpp 
    imgui/imgui_draw.cpp 
    imgui/imgui_impl_glfw.cpp 
    imgui/imgui_impl_glfw.h 
    imgui/imgui_impl_opengl3_loader.h
    imgui/imgui_impl_opengl3.cpp 
    imgui/imgui_impl_opengl3.h 
    imgui/imgui_internal.h 
    imgui/imgui_tables.cpp 
    imgui/imgui_widgets.cpp 
    imgui/imgui.cpp imgui/imgui.h 
    imgui/imstb_rectpack.h 
    imgui/imstb_textedit.h 
    imgui/imstb_truetype.h
) 

target_link_libraries(opengl_testing
    glfw 
    OpenGL::GL
)
