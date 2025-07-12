#pragma once

#include "imgui.h"
#include "backends/imgui_impl_glfw.h"
#include "backends/imgui_impl_opengl3.h"
#include <stdio.h>
#include <GL/glew.h>    // Initialize with glewInit()
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <cstdlib>

#include "PointProbabilityEngine.h"

class Viewer
{
private:
    PointProbabilityEngine *ppe;
    GLFWwindow *window;
    bool should_close = false;

    bool show_demo_window = true;
    bool show_another_window = false;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    ImGuiIO *io;

public:
    Viewer(PointProbabilityEngine *ppe);
    ~Viewer();

    void update();
    void render();

    bool getShouldClose() const { return should_close; }
};