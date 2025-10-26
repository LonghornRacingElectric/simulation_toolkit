#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "../glm/gtc/type_ptr.hpp"
#include "glad.h"
#include "GLFW/glfw3.h"
#include "sphere.h"
#include "rod.h"
#include "tire.h"
#include "imgui.h"

// === Global Variables ===
unsigned int shaderProgram;
glm::vec3 spherePosition(0.0f, 0.0f, 0.0f);

// Sphere state
int x = 0, y = 0, z = 0;
bool createNewNode = false;
std::vector<SphereInstance> spheres;

// Rod state
float xyzRodEnd1[3] = { -1.0f, 0.0f, 0.0f };
float xyzRodEnd2[3] = { 1.0f, 0.0f, 0.0f };
bool createNewRod = false;
std::vector<RodInstance> rods;

// Tire state
struct TireInstance {
    glm::vec3 position;
    float innerDiameter;
    float outerDiameter;
    float width;
    glm::vec4 color;
    unsigned int VAO, VBO, EBO;
    unsigned int indexCount;
};
std::vector<TireInstance> tires;

// Camera
float camAngleX = 0.0f, camAngleZ = 0.0f, camDistance = 5.0f;
float zoom = 1.0f;

// Selection state
int selectedNodeIndex = -1;  // -1 means no node selected
int selectedRodIndex = -1;  // -1 means no rod selected

// Recenter offset - set this to recenter the entire graph
glm::vec3 recenterOffset(-0.73f, 0.0f, 0.0f);

// === Recenter Function ===
glm::vec3 recenter(const glm::vec3& point, const glm::vec3& offset) {
    return glm::vec3(point.x - offset.x, point.y - offset.y, point.z - offset.z);
}

// === YAML Loading Function ===
void loadNodesAndRodsFromYAML(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cout << "Could not open " << filename << std::endl;
        return;
    }
    
    std::string line;
    bool inNodes = false, inRods = false, inTires = false;
    
    while (std::getline(file, line)) {
        if (line.find("nodes:") != std::string::npos) {
            inNodes = true;
            inRods = false;
            inTires = false;
            continue;
        }
        
        if (line.find("rods:") != std::string::npos) {
            inRods = true;
            inNodes = false;
            inTires = false;
            continue;
        }
        
        if (line.find("tires:") != std::string::npos) {
            inTires = true;
            inNodes = false;
            inRods = false;
            continue;
        }
        
        if (inNodes && line.find("position:") != std::string::npos) {
            // Extract numbers from [x, y, z] format
            size_t start = line.find('[') + 1;
            size_t end = line.find(']');
            std::string coords = line.substr(start, end - start);
            
            std::stringstream ss(coords);
            float x, y, z;
            char comma;
            ss >> x >> comma >> y >> comma >> z;
            
            glm::vec3 originalPos(x, y, z);
            glm::vec3 recenteredPos = recenter(originalPos, recenterOffset);
            spheres.push_back({ recenteredPos, 0.0f });
        }
        
        if (inRods && line.find("end1:") != std::string::npos) {
            // Get end1 coordinates
            size_t start = line.find('[') + 1;
            size_t end = line.find(']');
            std::string coords = line.substr(start, end - start);
            std::stringstream ss(coords);
            float x1, y1, z1;
            char comma;
            ss >> x1 >> comma >> y1 >> comma >> z1;
            
            // Get next line for end2
            std::getline(file, line);
            start = line.find('[') + 1;
            end = line.find(']');
            coords = line.substr(start, end - start);
            std::stringstream ss2(coords);
            float x2, y2, z2;
            ss2 >> x2 >> comma >> y2 >> comma >> z2;
            
            glm::vec3 end1Original(x1, y1, z1);
            glm::vec3 end2Original(x2, y2, z2);
            glm::vec3 end1Recentered = recenter(end1Original, recenterOffset);
            glm::vec3 end2Recentered = recenter(end2Original, recenterOffset);
            rods.push_back({ end1Recentered, end2Recentered });
        }
        
        if (inTires && line.find("contact_patch:") != std::string::npos) {
            // Get contact patch position
            size_t start = line.find('[') + 1;
            size_t end = line.find(']');
            std::string coords = line.substr(start, end - start);
            std::stringstream ss(coords);
            float cpx, cpy;
            char comma;
            ss >> cpx >> comma >> cpy;
            
            // Get inner_diameter
            std::getline(file, line);
            float innerDiam = std::stof(line.substr(line.find(':') + 1));
            
            // Get outer_diameter
            std::getline(file, line);
            float outerDiam = std::stof(line.substr(line.find(':') + 1));
            
            // Get width
            std::getline(file, line);
            float width = std::stof(line.substr(line.find(':') + 1));
            
            // Calculate tire center position from contact patch
            // Contact patch is at ground (z=0), tire center is at outerRadius above ground
            float outerRadius = outerDiam / 2.0f;
            glm::vec3 tireCenterOriginal(cpx, cpy, outerRadius);
            glm::vec3 tireCenterRecentered = recenter(tireCenterOriginal, recenterOffset);
            
            tires.push_back({ tireCenterRecentered, innerDiam, outerDiam, width, glm::vec4(0.1f, 0.1f, 0.1f, 1.0f), 0, 0, 0, 0 });
        }
    }
    
    file.close();
    std::cout << "Loaded " << spheres.size() << " nodes, " << rods.size() << " rods, and " << tires.size() << " tires from " << filename << std::endl;
}

// === Utility Functions ===
void framebuffer_size_callback(GLFWwindow* window, int width, int height) {
    glViewport(0, 0, width, height);
    glm::mat4 projection = glm::ortho(-3.0f, 3.0f, -2.0f, 2.0f, 0.1f, 100.0f);
    unsigned int projectionLoc = glGetUniformLocation(shaderProgram, "projection");
    glUniformMatrix4fv(projectionLoc, 1, GL_FALSE, glm::value_ptr(projection));
}

void setSpherePosition(float x_, float y_, float z_) {
    spherePosition = glm::vec3(x_, y_, z_);
}

int main() {
    // === GLFW + GLAD Initialization ===
    if (!glfwInit()) return -1;
    // === Request OpenGL 4.2 Core Profile (important for WSL compatibility) ===
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    // Optional: make the window forward-compatible (fixes macOS or strict drivers)
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(800, 600, "ImGui Rod System", NULL, NULL);
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) return -1;

    glEnable(GL_DEPTH_TEST);
    glClearColor(0.25f, 0.5f, 0.75f, 1.0f);
    initImGui(window);

    // === Geometry Setup ===
    float radius = 0.02f;
    unsigned int stacks = 20, slices = 20;
    float* sphereVertices = nullptr;
    unsigned int* sphereIndices = nullptr;
    unsigned int sphereVertCount = 0, sphereIndexCount = 0;
    generateSphere(radius, stacks, slices, sphereVertices, sphereIndices, sphereVertCount, sphereIndexCount);

    unsigned int VAO, VBO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sphereVertCount * 3 * sizeof(float), sphereVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sphereIndexCount * sizeof(unsigned int), sphereIndices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    float rodRadius = 0.01f, rodHeight = 2.0f;
    unsigned int rodSlices = 30, rodVAO, rodVBO, rodEBO;
    float* rodVertices = nullptr;
    unsigned int* rodIndices = nullptr;
    unsigned int rodVertexCount = 0, rodIndexCount = 0;
    generateRod(rodRadius, rodHeight, rodSlices, rodVertices, rodIndices, rodVertexCount, rodIndexCount);

    glGenVertexArrays(1, &rodVAO);
    glGenBuffers(1, &rodVBO);
    glGenBuffers(1, &rodEBO);
    glBindVertexArray(rodVAO);
    glBindBuffer(GL_ARRAY_BUFFER, rodVBO);
    glBufferData(GL_ARRAY_BUFFER, rodVertexCount * 3 * sizeof(float), rodVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, rodEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, rodIndexCount * sizeof(unsigned int), rodIndices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // === Plane Setup ===
    // Original plane vertices
    glm::vec3 planeCorners[] = {
        glm::vec3(-2.212f, -1.0f, 0.0f),
        glm::vec3( 0.788f, -1.0f, 0.0f),
        glm::vec3(-2.212f,  1.0f, 0.0f),
        glm::vec3( 0.788f,  1.0f, 0.0f)
    };
    
    // Apply recentering to plane vertices
    float planeVertices[12];
    for (int i = 0; i < 4; i++) {
        glm::vec3 recentered = recenter(planeCorners[i], recenterOffset);
        planeVertices[i * 3] = recentered.x;
        planeVertices[i * 3 + 1] = recentered.y;
        planeVertices[i * 3 + 2] = recentered.z;
    }
    unsigned int planeIndices[] = { 0, 1, 2, 1, 3, 2 };
    unsigned int planeVAO, planeVBO, planeEBO;
    glGenVertexArrays(1, &planeVAO);
    glGenBuffers(1, &planeVBO);
    glGenBuffers(1, &planeEBO);
    glBindVertexArray(planeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, planeVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(planeVertices), planeVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, planeEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(planeIndices), planeIndices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // === Shaders ===
    const char* vs = "#version 330 core\nlayout (location = 0) in vec3 aPos;\nuniform mat4 model, view, projection;\nvoid main() { gl_Position = projection * view * model * vec4(aPos, 1.0); }";
    const char* fs = "#version 330 core\n"
    "out vec4 FragColor;\n"
    "uniform vec4 objectColor;\n"
    "void main() {\n"
    "   FragColor = objectColor;\n"
    "}\0";

    unsigned int vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vs, NULL); glCompileShader(vertexShader);
    unsigned int fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fs, NULL); glCompileShader(fragmentShader);
    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    glUseProgram(shaderProgram);

    unsigned int modelLoc = glGetUniformLocation(shaderProgram, "model");
    unsigned int viewLoc = glGetUniformLocation(shaderProgram, "view");
    unsigned int projLoc = glGetUniformLocation(shaderProgram, "projection");
    unsigned int colorLoc = glGetUniformLocation(shaderProgram, "objectColor");

    glm::mat4 projection = glm::ortho(-3.0f, 3.0f, -2.0f, 2.0f, 0.1f, 100.0f);
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(projection));
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

    // Load from YAML FIRST
    loadNodesAndRodsFromYAML("model.yml");

    // === Tire Setup - Generate geometry for each tire AFTER loading from YAML ===
    unsigned int tireSegments = 40;
    for (auto& tire : tires) {
        float* tireVertices = nullptr;
        unsigned int* tireIndices = nullptr;
        unsigned int tireVertexCount = 0, tireIndexCount = 0;
        generateTire(tire.innerDiameter, tire.outerDiameter, tire.width, tireSegments,
                     tireVertices, tireIndices, tireVertexCount, tireIndexCount);
        
        unsigned int tireVAO, tireVBO, tireEBO;
        glGenVertexArrays(1, &tireVAO);
        glGenBuffers(1, &tireVBO);
        glGenBuffers(1, &tireEBO);
        glBindVertexArray(tireVAO);
        glBindBuffer(GL_ARRAY_BUFFER, tireVBO);
        glBufferData(GL_ARRAY_BUFFER, tireVertexCount * 3 * sizeof(float), tireVertices, GL_STATIC_DRAW);
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tireEBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, tireIndexCount * sizeof(unsigned int), tireIndices, GL_STATIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);
        
        tire.VAO = tireVAO;
        tire.VBO = tireVBO;
        tire.EBO = tireEBO;
        tire.indexCount = tireIndexCount;
        
        delete[] tireVertices;
        delete[] tireIndices;
    }

    // === Main Loop ===
    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        handleKeyboardInput(window);  // Handle keyboard input for camera controls
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        renderImGuiControls();

        // X rotation for camera position (orbit), Z rotation for camera orientation (roll)
        float radX = glm::radians(camAngleX), radZ = glm::radians(camAngleZ);
        
        // Start with camera at distance along Z-axis
        glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, camDistance);
        
        // Apply only X rotation to camera position (orbit around target)
        glm::mat4 rotationX = glm::rotate(glm::mat4(1.0f), radX, glm::vec3(1.0f, 0.0f, 0.0f));
        cameraPos = glm::vec3(rotationX * glm::vec4(cameraPos, 1.0f));

        // Create the view matrix using the X-rotated camera position
        glm::mat4 view = glm::lookAt(cameraPos, glm::vec3(0), glm::vec3(0, 1, 0));
        
        // Apply Z rotation (roll) to the view matrix - this only affects camera orientation, not position
        if (abs(camAngleZ) > 0.001f) {
            glm::vec3 forward = glm::normalize(glm::vec3(0, 0, 0) - cameraPos);
            view = glm::rotate(view, radZ, forward);
        }

        view = view * glm::scale(glm::mat4(1.0f), glm::vec3(zoom));
        glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));

        if (createNewNode) {
            spheres.push_back({ spherePosition, 0.0f });
            createNewNode = false;
        }

        if (createNewRod) {
            rods.push_back({ glm::vec3(xyzRodEnd1[0], xyzRodEnd1[1], xyzRodEnd1[2]),
                             glm::vec3(xyzRodEnd2[0], xyzRodEnd2[1], xyzRodEnd2[2]) });
            createNewRod = false;
        }

        // === Draw Plane ===
        glBindVertexArray(planeVAO);
        glm::mat4 planeModel = glm::mat4(1.0f);
        glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(planeModel));
        glUniform4f(colorLoc, 0.5f, 0.5f, 0.5f, 1.0f);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

        // === Draw Tire ===
        for (auto& t : tires) {
            glBindVertexArray(t.VAO);
            glm::mat4 tireModel = glm::translate(glm::mat4(1.0f), t.position);
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(tireModel));
            glUniform4fv(colorLoc, 1, glm::value_ptr(t.color));
            glDrawElements(GL_TRIANGLES, t.indexCount, GL_UNSIGNED_INT, 0);
        }

        // === Draw Spheres ===
        glBindVertexArray(VAO);
        for (int i = 0; i < spheres.size(); i++) {
            glm::mat4 model = glm::translate(glm::mat4(1.0f), spheres[i].position);
            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            
            // Highlight selected node in blue
            if (i == selectedNodeIndex) {
                glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f); // Blue
            } else {
                glUniform4fv(colorLoc, 1, glm::value_ptr(spheres[i].color));
            }
            glDrawElements(GL_TRIANGLES, sphereIndexCount, GL_UNSIGNED_INT, 0);
        }

        // === Draw Rods ===
        glBindVertexArray(rodVAO);
        for (int i = 0; i < rods.size(); i++) {
            auto& r = rods[i];
            // Calculate direction, length, and midpoint for the rod
            glm::vec3 dir = r.end2 - r.end1;
            float length = glm::length(dir);
            glm::vec3 mid = (r.end1 + r.end2) * 0.5f;

            // Normalize direction vector
            glm::vec3 dirNormalized = glm::normalize(dir);

            // Calculate rotation axis and angle to align the rod with the correct direction
            glm::vec3 axis = glm::cross(glm::vec3(0, 1, 0), dirNormalized);
            float angle = acos(glm::clamp(glm::dot(glm::vec3(0, 1, 0), dirNormalized), -1.0f, 1.0f));

            // Apply the transformation to the rod: translation, rotation, scaling
            glm::mat4 model = glm::translate(glm::mat4(1.0f), mid);
            if (glm::length(axis) > 0.001f) {
                model = glm::rotate(model, angle, glm::normalize(axis));
            }
            model = glm::scale(model, glm::vec3(1.0f, length / rodHeight, 1.0f));

            glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
            
            // Highlight selected rod in blue
            if (i == selectedRodIndex) {
                glUniform4f(colorLoc, 0.0f, 0.0f, 1.0f, 1.0f); // Blue
            } else {
                glUniform4fv(colorLoc, 1, glm::value_ptr(r.color));
            }

            glDrawElements(GL_TRIANGLES, rodIndexCount, GL_UNSIGNED_INT, 0);
        }

        // Render ImGui UI controls
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    // Cleanup
    cleanupImGui();
    glfwTerminate();
    delete[] sphereVertices;
    delete[] sphereIndices;
    delete[] rodVertices;
    delete[] rodIndices;
    
    for (auto& tire : tires) {
        glDeleteVertexArrays(1, &tire.VAO);
        glDeleteBuffers(1, &tire.VBO);
        glDeleteBuffers(1, &tire.EBO);
    }

    return 0;
}