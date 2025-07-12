#include "FloorPlane.h"

FloorPlane::FloorPlane()
{
    std::vector<float> line_vertices;
    std::vector<unsigned int> line_indices;

    for (int i = -10; i <= 10; i++)
    {
        // Vertical lines
        line_vertices.push_back(i);
        line_vertices.push_back(0.0f);
        line_vertices.push_back(-10.0f);
        line_vertices.push_back(i);
        line_vertices.push_back(0.0f);
        line_vertices.push_back(10.0f);

        // Horizontal lines
        line_vertices.push_back(-10.0f);
        line_vertices.push_back(0.0f);
        line_vertices.push_back(i);
        line_vertices.push_back(10.0f);
        line_vertices.push_back(0.0f);
        line_vertices.push_back(i);
    }

    for (int i = 0; i < 21; i++)
    {
        // Vertical lines indices
        line_indices.push_back(i * 2);
        line_indices.push_back(i * 2 + 1);

        // Horizontal lines indices
        line_indices.push_back(42 + i * 2);
        line_indices.push_back(42 + i * 2 + 1);
    }

    GLuint lineVBO, lineEBO = 0;

    glGenVertexArrays(1, &linesVAO);
    glBindVertexArray(linesVAO);

    glGenBuffers(1, &lineVBO);
    glGenBuffers(1, &lineEBO);

    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, line_vertices.size() * sizeof(float), line_vertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, lineEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, line_indices.size() * sizeof(unsigned int), line_indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void FloorPlane::draw() const
{
    glBindVertexArray(linesVAO);

    ShaderManager *sm = ShaderManager::getInstance();
    sm->useShader("solid_color_3d");
    sm->setModelMatrix(modelPose);
    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_FALSE); // Disable depth writing for transparency
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    sm->setColor(glm::vec4(0.3f, 0.3f, 0.3f, 0.1f));  // Set a gray color for the grid lines
    glDrawElements(GL_LINES, 84, GL_UNSIGNED_INT, 0); // 42 vertical + 42 horizontal = 84 indices
    glDepthMask(GL_TRUE);                             // Re-enable depth writing

    glBindVertexArray(0);
}