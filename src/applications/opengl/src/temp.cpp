#include <iostream>
#include <cmath>
#include <array>
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "../glm/gtc/type_ptr.hpp"
#include "glad.h"
#include "GLFW/glfw3.h"
#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "cube.h"

// Rotation matrix functions (same as previous)
glm::mat4 rotateX(float angle) {
    glm::mat4 rotation = glm::mat4(1.0f);
    rotation[1][1] = cos(angle);
    rotation[1][2] = -sin(angle);
    rotation[2][1] = sin(angle);
    rotation[2][2] = cos(angle);
    return rotation;
}

glm::mat4 rotateY(float angle) {
    glm::mat4 rotation = glm::mat4(1.0f);
    rotation[0][0] = cos(angle);
    rotation[0][2] = sin(angle);
    rotation[2][0] = -sin(angle);
    rotation[2][2] = cos(angle);
    return rotation;
}

glm::mat4 rotateZ(float angle) {
    glm::mat4 rotation = glm::mat4(1.0f);
    rotation[0][0] = cos(angle);
    rotation[0][1] = -sin(angle);
    rotation[1][0] = sin(angle);
    rotation[1][1] = cos(angle);
    return rotation;
}

// Called whenever window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
}

void resetSlider(float* slider){
    if (*slider <= -1.0f){
        *slider = 1.0f;
    } else if (*slider >= 1.0f){
        *slider = -1.0f;
    }
}

float zoom = 1.0f;
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset){
    if (yoffset > 0){
        zoom += 0.1f;
    } else if (yoffset < 0){
        zoom -= 0.1f;
    }
    zoom = std::max(0.1f, std::min(zoom, 1.5f));  // Clamp zoom between 0.1 and 1.5
}

int main() {
    // ----Setup----
    GLFWwindow* window;

    // Ensures GLFW library is initialized
    if (!glfwInit()) {
        return -1;
    }

    // Set GLFW context hints
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window and OpenGL context
    window = glfwCreateWindow(640, 640, "Window", NULL, NULL);
    if (!window) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Specifies this is the window we are working in the context of
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Initialize GLAD
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cout << "Couldn't load OpenGL" << std::endl;
        glfwTerminate();
        return -1;
    }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // ----Window Elements----
    // Colors
    ImGui::StyleColorsDark();
    glClearColor(0.25f, 0.5f, 0.75f, 1.0f);

    // XYZ slider values
    float x_slider = -0.05f;
    float y_slider = -0.05f;
    float z_slider = 0.0f;
    float angleIncrement = 0.01f;

    // Set up initial projection matrix
    glm::mat4 projection = glm::ortho(-1.0f, 1.0f, -1.0f, 1.0f, -1.0f, 1.0f);

    // Create a VAO, VBO, and IBO for the cube
    unsigned int VAO, VBO, IBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &IBO);

    // Bind the VAO and VBO, and set the vertex data for the cube
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(Cube::xyzArray), Cube::xyzArray.data(), GL_STATIC_DRAW);

    // Bind the IBO and set the index data
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, IBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(Cube::triangleVertexIndices), Cube::triangleVertexIndices.data(), GL_STATIC_DRAW);

    // Set up vertex attribute pointer
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0); // Position attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1); // Color attribute
    glBindVertexArray(0);

    // Create shader program (vertex & fragment shaders)
    const char* vertexShaderSource = R"(
#version 330 core
layout(location = 0) in vec3 position;
layout(location = 1) in vec3 color;
out vec3 fragColor;
uniform mat4 transform;
uniform mat4 projection;
void main()
{
    gl_Position = projection * transform * vec4(position, 1.0);
    fragColor = color;
}
)";
    const char* fragmentShaderSource = R"(
#version 330 core
in vec3 fragColor;
out vec4 color;
void main()
{
    color = vec4(fragColor, 1.0);
}
)";

    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glUseProgram(shaderProgram);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);  // Enables depth testing

    // Set the depth function to prefer smaller Z values (closer objects are rendered first)
    glDepthFunc(GL_LESS);  // Default: smaller values are in front

    // Register scroll callback to handle zoom
    glfwSetScrollCallback(window, scroll_callback);

    // Main Loop
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render ImGui UI
        ImGui::Begin("Cube Control");

        ImGui::SliderFloat("X-Axis", &x_slider, -1.0f, 1.0f);
        ImGui::SameLine();
        if (ImGui::Button("Reset X")) {
            x_slider = 0.0f;
        }

        ImGui::SliderFloat("Y-Axis", &y_slider, -1.0f, 1.0f);
        ImGui::SameLine();
        if (ImGui::Button("Reset Y")) {
            y_slider = 0.0f;
        }

        ImGui::SliderFloat("Z-Axis", &z_slider, -1.0f, 1.0f);
        ImGui::SameLine();
        if (ImGui::Button("Reset Z")) {
            z_slider = 0.0f;
        }

        ImGui::Text("X: %.3f, Y: %.3f, Z: %.3f", x_slider, y_slider, z_slider);

        if (ImGui::Button("Reset All")) {
            x_slider = 0.0f;
            y_slider = 0.0f;
            z_slider = 0.0f;
        }

        ImGui::End();

        // Apply zoom to the scaling matrix (multiply zoom with the cube dimensions)
        glm::mat4 scalingMatrix = glm::mat4(1.0f);
        scalingMatrix = glm::scale(scalingMatrix, glm::vec3(zoom, zoom, zoom));

        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            y_slider += angleIncrement;
        } else if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            y_slider -= angleIncrement;
        }

        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            x_slider -= angleIncrement;
        } else if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            x_slider += angleIncrement;
        }

        resetSlider(&x_slider);
        resetSlider(&y_slider);
        resetSlider(&z_slider);

        // Apply rotation matrix based on sliders
        glm::mat4 rotationMatrix = glm::mat4(1.0f); // Identity matrix
        rotationMatrix = rotateX(glm::radians(x_slider * 180.0f)) * rotationMatrix;
        rotationMatrix = rotateY(glm::radians(y_slider * 180.0f)) * rotationMatrix;
        rotationMatrix = rotateZ(glm::radians(z_slider * 180.0f)) * rotationMatrix;

        glm::mat4 transform = scalingMatrix * rotationMatrix;

        // Adjust projection matrix based on the window size
        int width, height;
        glfwGetFramebufferSize(window, &width, &height);
        float aspectRatio = float(width) / float(height);
        projection = glm::ortho(-1.0f * aspectRatio, 1.0f * aspectRatio, -1.0f, 1.0f, -1.0f, 1.0f);

        // Set the transform and projection uniform in the shader
        unsigned int transformLoc = glGetUniformLocation(shaderProgram, "transform");
        glUniformMatrix4fv(transformLoc, 1, GL_FALSE, &transform[0][0]);
        unsigned int projectionLoc = glGetUniformLocation(shaderProgram, "projection");
        glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, &projection[0][0]);

        // Clear the screen and depth buffer
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw the cube using elements (IBO)
        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, Cube::triangleVertexIndices.size(), GL_UNSIGNED_INT, 0);

        // Render ImGui UI
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup and close
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &IBO);

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwTerminate();

    return 0;
}
