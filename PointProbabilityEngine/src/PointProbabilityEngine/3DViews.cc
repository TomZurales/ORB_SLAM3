#include "PointProbabilityEngine.h"

#include <GL/glew.h>

void PointProbabilityEngine::init3DView()
{
    // Generate framebuffer
    glGenFramebuffers(1, &camera_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, camera_fbo);
    glObjectLabel(GL_FRAMEBUFFER, camera_fbo, -1, "3D View FBO");
    // Generate texture to render to
    glGenTextures(1, &camera_texture);
    glBindTexture(GL_TEXTURE_2D, camera_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1000, 1000, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, camera_texture, 0);

    // Create and attach a renderbuffer for depth and stencil
    glGenRenderbuffers(1, &camera_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, camera_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 1000, 1000);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, camera_rbo);

    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "Error: Framebuffer is not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return;
    }

    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    // Generate framebuffer
    glGenFramebuffers(1, &static_fbo);
    glBindFramebuffer(GL_FRAMEBUFFER, static_fbo);
    glObjectLabel(GL_FRAMEBUFFER, static_fbo, -1, "Static View FBO");
    // Generate texture to render to
    glGenTextures(1, &static_texture);
    glBindTexture(GL_TEXTURE_2D, static_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1000, 1000, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, static_texture, 0);

    // Create and attach a renderbuffer for depth and stencil
    glGenRenderbuffers(1, &static_rbo);
    glBindRenderbuffer(GL_RENDERBUFFER, static_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, 1000, 1000);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, static_rbo);

    // Check framebuffer completeness
    if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    {
        std::cerr << "Error: Framebuffer is not complete!" << std::endl;
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        return;
    }

    // Unbind framebuffer
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ShaderManager *sm = ShaderManager::getInstance();
    // Add our shaders for the camera view
    sm->addShader("solid_color_3d",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/vertex/solid_color_3d.glsl",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/fragment/solid_color_3d.glsl");
    sm->addShader("heatmap_3d",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/vertex/solid_color_3d.glsl",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/fragment/heatmap_3d.glsl");

    // Add our shaders for the static view
    sm->addShader("static_solid_color_3d",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/vertex/static_solid_color_3d.glsl",
                  "/home/tom/workspace/thesis/code/src/Graphics/shaders/fragment/solid_color_3d.glsl");

    sm->setProjectionMatrix(glm::perspective(glm::radians(45.0f), 1.0f, 0.01f, 100.0f));
    sm->setChangeOfBasis(glm::scale(glm::mat4(1.0f), glm::vec3(1.0f, -1.0f, 1.0f)));
    // sm->setChangeOfBasis(glm::mat4(1.0f));

    floorPlane = new FloorPlane();
    pointCloud = new PointCloud();
    icosModel = new IcosModel();
    cameraModel = new CameraModel();
}

void PointProbabilityEngine::showCameraView() const
{
    ImGui::Begin("Camera View");
    ImVec2 windowSize = ImGui::GetContentRegionAvail();
    glBindFramebuffer(GL_FRAMEBUFFER, camera_fbo);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, windowSize.x, windowSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, camera_texture, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, camera_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, windowSize.x, windowSize.y);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, camera_rbo);

    glViewport(0, 0, windowSize.x, windowSize.y);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ShaderManager::getInstance()->setViewMatrix(eigenToGlm(backend->getLastCameraPose().inverse()));

    pointCloud->draw(backend->getAllPoints());

    // Draw the floor plane last (transparent objects)
    floorPlane->draw();

    icosModel->draw(backend->getActivePoint(), ((IcosahedronBackend *)backend)->getActiveIcosahedron()); // Draw the icosahedron model at the origin
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ImGui::Image((ImTextureID)camera_texture, windowSize); // Placeholder for 3D rendering
    ImGui::End();
}

void PointProbabilityEngine::showStaticView()
{
    ImGui::Begin("Static View");

    // Handle mouse input for camera control
    if (ImGui::IsWindowHovered())
    {
        ImGuiIO &io = ImGui::GetIO();
        static glm::vec2 lastMousePos(0.0f, 0.0f);
        static bool isDragging = false;
        static bool isTranslating = false;
        static glm::vec3 cameraTarget(0.0f, 0.0f, 0.0f); // Center point to orbit around
        static float cameraDistance = 5.0f;              // Distance from target
        static float cameraYaw = 0.0f;                   // Horizontal rotation
        static float cameraPitch = 0.0f;                 // Vertical rotation
        static bool initialized = false;

        // Initialize from existing staticViewMatrix on first hover
        if (!initialized)
        {
            // Extract camera position and target from existing view matrix
            glm::mat4 invView = glm::inverse(staticViewMatrix);
            glm::vec3 cameraPos = glm::vec3(invView[3]);

            // Assume looking at origin for now, can be improved
            cameraTarget = glm::vec3(0.0f, 0.0f, 0.0f);
            cameraDistance = glm::length(cameraPos - cameraTarget);

            // Calculate yaw and pitch from camera position
            glm::vec3 direction = glm::normalize(cameraPos - cameraTarget);
            cameraYaw = atan2(direction.z, direction.x);
            cameraPitch = asin(direction.y);

            initialized = true;
        }

        glm::vec2 currentMousePos(io.MousePos.x, io.MousePos.y);

        // Handle mouse wheel for zooming (increased sensitivity)
        if (io.MouseWheel != 0.0f)
        {
            cameraDistance = glm::max(0.1f, cameraDistance - io.MouseWheel * 1.0f);
        }

        // Handle left mouse button for orbit rotation
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left))
        {
            isDragging = true;
            lastMousePos = currentMousePos;
        }
        else if (ImGui::IsMouseClicked(ImGuiMouseButton_Middle))
        {
            isTranslating = true;
            lastMousePos = currentMousePos;
        }

        if (ImGui::IsMouseReleased(ImGuiMouseButton_Left))
        {
            isDragging = false;
        }
        else if (ImGui::IsMouseReleased(ImGuiMouseButton_Middle))
        {
            isTranslating = false;
        }

        if (isDragging && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
        {
            glm::vec2 delta = (currentMousePos - lastMousePos) * 0.005f; // Decreased sensitivity

            cameraYaw += delta.x;
            cameraPitch = glm::clamp(cameraPitch + delta.y, -glm::pi<float>() * 0.49f, glm::pi<float>() * 0.49f);

            lastMousePos = currentMousePos;
        }
        else if (isTranslating && ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
        {
            glm::vec2 delta = (currentMousePos - lastMousePos) * 0.01f;

            // Pan the target point
            glm::vec3 right = glm::normalize(glm::cross(glm::vec3(0.0f, 1.0f, 0.0f),
                                                        glm::vec3(cos(cameraPitch) * cos(cameraYaw),
                                                                  sin(cameraPitch),
                                                                  cos(cameraPitch) * sin(cameraYaw))));
            glm::vec3 up = glm::normalize(glm::cross(glm::vec3(cos(cameraPitch) * cos(cameraYaw),
                                                               sin(cameraPitch),
                                                               cos(cameraPitch) * sin(cameraYaw)),
                                                     right));

            cameraTarget += right * delta.x * cameraDistance * 0.1f - up * delta.y * cameraDistance * 0.1f;

            lastMousePos = currentMousePos;
        }

        // Calculate camera position based on spherical coordinates
        glm::vec3 cameraPos = cameraTarget + glm::vec3(
                                                 cameraDistance * cos(cameraPitch) * cos(cameraYaw),
                                                 cameraDistance * sin(cameraPitch),
                                                 cameraDistance * cos(cameraPitch) * sin(cameraYaw));

        // Create view matrix looking at the target
        staticViewMatrix = glm::lookAt(cameraPos, cameraTarget, glm::vec3(0.0f, 1.0f, 0.0f));
    }
    ImVec2 windowSize = ImGui::GetContentRegionAvail();
    glBindFramebuffer(GL_FRAMEBUFFER, static_fbo);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, windowSize.x, windowSize.y, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, static_texture, 0);
    glBindRenderbuffer(GL_RENDERBUFFER, static_rbo);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH24_STENCIL8, windowSize.x, windowSize.y);
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, static_rbo);

    glViewport(0, 0, windowSize.x, windowSize.y);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    ShaderManager::getInstance()->setViewMatrix(staticViewMatrix);

    pointCloud->draw(backend->getAllPoints());
    cameraModel->draw(backend->getLastCameraPose()); // Draw the camera model at the origin

    // Draw the floor plane last (transparent objects)
    floorPlane->draw();

    icosModel->draw(backend->getActivePoint(), ((IcosahedronBackend *)backend)->getActiveIcosahedron()); // Draw the icosahedron model at the origin

    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    ImGui::Image((ImTextureID)static_texture, windowSize);
    ImGui::End();
}