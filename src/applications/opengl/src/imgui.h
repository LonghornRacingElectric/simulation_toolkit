#ifndef IMGUI_CONTROLS_H
#define IMGUI_CONTROLS_H

#include "../imgui/imgui.h"
#include "../imgui/imgui_impl_glfw.h"
#include "../imgui/imgui_impl_opengl3.h"
#include "../glm/glm.hpp"
#include "../glm/gtc/matrix_transform.hpp"
#include "../glm/gtc/type_ptr.hpp"
#include <vector>
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
extern float camAngleX, camAngleY, camAngleZ, camDistance, zoom;
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
    ImGui::SliderFloat("Orbit Y (left/right)", &camAngleY, 0.0f, 360.0f);
    ImGui::SliderFloat("Orbit Z (roll)", &camAngleZ, -180.0f, 180.0f);  // Add this line
    ImGui::SliderFloat("Zoom (distance)", &zoom, 0.1f, 3.0f);

    ImGui::End();

    // === Node Manager Panel ===
    ImGui::Begin("Node Info");

    if (ImGui::Button("Clear All Nodes")) {
        spheres.clear();
    }

    for (int i = 0; i < static_cast<int>(spheres.size()); ++i) {
        std::string header = "Node " + std::to_string(i);
        if (ImGui::CollapsingHeader(header.c_str())) {
            ImGui::PushID(i);

            ImGui::DragFloat3("Position", glm::value_ptr(spheres[i].position), 0.1f);
            ImGui::ColorEdit4("Color", glm::value_ptr(spheres[i].color));

            if (ImGui::Button("Delete Node")) {
                spheres.erase(spheres.begin() + i);
                ImGui::PopID();
                break;
            }

            ImGui::PopID();
        }
    }

    ImGui::End();

    // === Rod Manager Panel ===
    ImGui::Begin("Rod Info");

    if (ImGui::Button("Clear All Rods")) {
        rods.clear();
    }

    for (int i = 0; i < static_cast<int>(rods.size()); ++i) {
        std::string header = "Rod " + std::to_string(i);
        if (ImGui::CollapsingHeader(header.c_str())) {
            ImGui::PushID(i);

            ImGui::DragFloat3("Start", glm::value_ptr(rods[i].end1), 0.1f);
            ImGui::DragFloat3("End", glm::value_ptr(rods[i].end2), 0.1f);
            ImGui::ColorEdit4("Color", glm::value_ptr(rods[i].color));

            if (ImGui::Button("Delete Rod")) {
                rods.erase(rods.begin() + i);
                ImGui::PopID();
                break;
            }

            ImGui::PopID();
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
