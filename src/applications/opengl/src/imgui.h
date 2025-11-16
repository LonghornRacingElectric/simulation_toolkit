#ifndef IMGUI_CONTROLS_H
#define IMGUI_CONTROLS_H

#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "../glm/gtc/type_ptr.hpp"
#include <vector>
#include <float.h>
#include "GLFW/glfw3.h"  // For GLFWwindow
#include "sphere.h"

// --------- Forward Declarations & Shared State --------- //
void setSpherePosition(float x, float y, float z);

struct SphereInstance {
    glm::vec3 position;
    float rotationAngle;
    glm::vec4 color = glm::vec4(0.2f, 0.0f, 0.5f, 1.0f);
};

struct RodInstance {
    glm::vec3 end1;
    glm::vec3 end2;
    glm::vec4 color = glm::vec4(0.25f, 0.25f, 0.25f, 1.0f);
};

// Extern variables declared in main.cpp
extern int x, y, z;
extern bool createNewNode;
extern bool createNewRod;
extern float camAngleX, camAngleZ, camDistance, zoom;
extern int selectedNodeIndex, selectedRodIndex;
extern glm::vec3 recenterOffset;
extern float xyzRodEnd1[3];
extern float xyzRodEnd2[3];
extern std::vector<SphereInstance> spheres;
extern std::vector<RodInstance> rods;

// --------- ImGui Initialization --------- //
void initImGui(GLFWwindow* window) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
}

// --------- Keyboard Input Handling --------- //
// Handle object selection with Shift+Click
void handleObjectSelection(GLFWwindow* window) {
    // Only handle Shift+Click for object selection
    bool shiftPressed = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                       glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS;
    
    if (shiftPressed && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS) {
        static bool wasPressed = false;
        if (!wasPressed) { // Only trigger on press, not hold
            wasPressed = true;
            
            double xpos, ypos;
            glfwGetCursorPos(window, &xpos, &ypos);
            
            int width, height;
            glfwGetWindowSize(window, &width, &height);
            
            // Convert screen coordinates to normalized device coordinates
            float x = (2.0f * xpos) / width - 1.0f;
            float y = 1.0f - (2.0f * ypos) / height;
            
            // Simple selection logic - find closest object to mouse position
            float minDistance = 0.3f; // Selection threshold
            int closestNode = -1;
            int closestRod = -1;
            float closestNodeDist = FLT_MAX; // Start with very large distance
            float closestRodDist = FLT_MAX;  // Start with very large distance
            
            // Check nodes (spheres) - use actual world positions
            for (int i = 0; i < spheres.size(); i++) {
                glm::vec3 worldPos = spheres[i].position;
                // Convert to screen space (very simplified - assumes orthographic projection)
                float screenX = worldPos.x * 0.3f; // Scale factor for ortho projection
                float screenY = worldPos.y * 0.3f;
                
                float distance = sqrt((x - screenX) * (x - screenX) + (y - screenY) * (y - screenY));
                if (distance < closestNodeDist) {
                    closestNode = i;
                    closestNodeDist = distance;
                }
            }
            
            // Check rods
            for (int i = 0; i < rods.size(); i++) {
                glm::vec3 mid = (rods[i].end1 + rods[i].end2) * 0.5f;
                float screenX = mid.x * 0.3f;
                float screenY = mid.y * 0.3f;
                
                float distance = sqrt((x - screenX) * (x - screenX) + (y - screenY) * (y - screenY));
                if (distance < closestRodDist) {
                    closestRod = i;
                    closestRodDist = distance;
                }
            }
            
            // Always clear selections first
            selectedNodeIndex = -1;
            selectedRodIndex = -1;
            
            // Select the closest object (node or rod) if it's within threshold
            if (closestNodeDist < closestRodDist && closestNodeDist < minDistance) {
                selectedNodeIndex = closestNode;
            } else if (closestRodDist < minDistance) {
                selectedRodIndex = closestRod;
            }
            // If no object is close enough, selections remain cleared (deselection)
        }
    } else {
        static bool wasPressed = false;
        wasPressed = false; // Reset when button is released or shift is not pressed
    }
}

void handleKeyboardInput(GLFWwindow* window) {
    // Arrow key controls for camera
    float step = 2.0f; // Step size for camera angle changes
    float zoomStep = 0.1f; // Step size for zoom changes
    
    // Check if Shift is pressed
    bool shiftPressed = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || 
                        glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    
    if (shiftPressed) {
        // Shift + arrow keys for Z axis and zoom controls
        if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS) {
            camAngleZ -= step;
            if (camAngleZ < -180.0f) camAngleZ += 360.0f;
        }
        if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS) {
            camAngleZ += step;
            if (camAngleZ > 180.0f) camAngleZ -= 360.0f;
        }
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            zoom += zoomStep;
            if (zoom > 3.0f) zoom = 3.0f;
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            zoom -= zoomStep;
            if (zoom < 0.1f) zoom = 0.1f;
        }
    } else {
        // Regular arrow keys for X axis controls only (limited to -89 to 89 degrees)
        if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS) {
            camAngleX += step;
            if (camAngleX > 89.0f) camAngleX = 89.0f;
        }
        if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS) {
            camAngleX -= step;
            if (camAngleX < -89.0f) camAngleX = -89.0f;
        }
    }
}

// --------- ImGui UI Controls --------- //
void renderImGuiControls() {
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();

    // === Control Panel ===
    ImGui::Begin("Control Panel");

    if (ImGui::Button("New Node", ImVec2(100, 25))) {
        createNewNode = true;
    }

    if (ImGui::Button("New Rod", ImVec2(100, 25))) {
        createNewRod = true;
    }

    ImGui::Separator();
    ImGui::Text("Camera Controls");
    ImGui::SliderFloat("Orbit X (up/down)", &camAngleX, -89.0f, 89.0f);
    ImGui::SliderFloat("Orbit Z (roll)", &camAngleZ, 0.0f, 360.0f);  
    ImGui::SliderFloat("Zoom (distance)", &zoom, 0.1f, 3.0f);
    
    ImGui::Separator();
    ImGui::Text("Recenter Controls");
    ImGui::DragFloat3("Recenter Offset", glm::value_ptr(recenterOffset), 0.01f);
    ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.0f, 1.0f), "Note: Recenter offset is applied when loading YAML");
    
    ImGui::Separator();
    ImGui::TextColored(ImVec4(0.0f, 0.0f, 1.0f, 1.0f), "Click on items in dropdowns to select");

    ImGui::End();

    // === Node Manager Panel ===
    ImGui::Begin("Node Info");

    if (ImGui::Button("Clear All Nodes")) {
        spheres.clear();
        selectedNodeIndex = -1; // Clear selection when clearing all nodes
    }

    for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
        std::string header = "Node " + std::to_string(i);
        bool isSelected = (i == selectedNodeIndex);
        
        // Highlight selected node with blue background
        if (isSelected) {
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.0f, 0.0f, 1.0f, 0.3f)); // Blue background
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.0f, 0.0f, 1.0f, 0.5f)); // Brighter blue on hover
        }
        
        // Make the header clickable for selection
        if (ImGui::Selectable(header.c_str(), isSelected, ImGuiSelectableFlags_AllowItemOverlap)) {
            // Clear other selections and select this node
            selectedNodeIndex = i;
            selectedRodIndex = -1;
        }
        
        // Show properties if this node is selected
        if (isSelected) {
            ImGui::Indent();
            ImGui::DragFloat3("Position", glm::value_ptr(spheres[i].position), 0.1f);
            ImGui::ColorEdit4("Color", glm::value_ptr(spheres[i].color));
            
            if (ImGui::Button("Delete Node")) {
                spheres.erase(spheres.begin() + i);
                selectedNodeIndex = -1; // Clear selection if deleting selected node
                ImGui::Unindent();
                // Pop the blue background colors before breaking
                ImGui::PopStyleColor(2);
                break;
            }
            ImGui::Unindent();
        }
        
        // Pop the blue background colors if this was the selected node
        if (isSelected) {
            ImGui::PopStyleColor(2);
        }
    }

    ImGui::End();

    // === Rod Manager Panel ===
    ImGui::Begin("Rod Info");

    if (ImGui::Button("Clear All Rods")) {
        rods.clear();
        selectedRodIndex = -1; // Clear selection when clearing all rods
    }

    for (int i = 0; i < static_cast<int>(rods.size()); ++i) {
        std::string header = "Rod " + std::to_string(i);
        bool isSelected = (i == selectedRodIndex);
        
        // Highlight selected rod with blue background
        if (isSelected) {
            ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(0.0f, 0.0f, 1.0f, 0.3f)); // Blue background
            ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(0.0f, 0.0f, 1.0f, 0.5f)); // Brighter blue on hover
        }
        
        // Make the header clickable for selection
        if (ImGui::Selectable(header.c_str(), isSelected, ImGuiSelectableFlags_AllowItemOverlap)) {
            // Clear other selections and select this rod
            selectedRodIndex = i;
            selectedNodeIndex = -1;
        }
        
        // Show properties if this rod is selected
        if (isSelected) {
            ImGui::Indent();
            ImGui::DragFloat3("Start", glm::value_ptr(rods[i].end1), 0.1f);
            ImGui::DragFloat3("End", glm::value_ptr(rods[i].end2), 0.1f);
            ImGui::ColorEdit4("Color", glm::value_ptr(rods[i].color));
            
            if (ImGui::Button("Delete Rod")) {
                rods.erase(rods.begin() + i);
                selectedRodIndex = -1; // Clear selection if deleting selected rod
                ImGui::Unindent();
                // Pop the blue background colors before breaking
                ImGui::PopStyleColor(2);
                break;
            }
            ImGui::Unindent();
        }
        
        // Pop the blue background colors if this was the selected rod
        if (isSelected) {
            ImGui::PopStyleColor(2);
        }
    }

    ImGui::End();

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// --------- Cleanup --------- //
void cleanupImGui() {
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
}

#endif // IMGUI_CONTROLS_H
