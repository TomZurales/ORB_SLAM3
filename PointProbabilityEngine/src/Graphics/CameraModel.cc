#include "CameraModel.h"

CameraModel::CameraModel()
{
    GLuint VBO, EBO = 0;

    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    std::vector<float> pointData =
        {
            0.0f, 0.0f, 0.0f,    // Camera center
            1.0f, 1.0f, -1.0f,   // Top right corner
            -1.0f, 1.0f, -1.0f,  // Top left corner
            -1.0f, -1.0f, -1.0f, // Bottom left corner
            1.0f, -1.0f, -1.0f   // Bottom right corner
        };
    std::vector<unsigned int> indices =
        {
            0, 1,
            0, 2,
            0, 3,
            0, 4,
            1, 2,
            2, 3,
            3, 4,
            4, 1};
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, pointData.size() * sizeof(float), pointData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0); // Unbind EBO
}

void CameraModel::draw(Eigen::Matrix4f cameraPose) const
{
    glBindVertexArray(VAO);

    ShaderManager *sm = ShaderManager::getInstance();
    sm->useShader("solid_color_3d");
    sm->setColor(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Black color

    sm->setModelMatrix(eigenToGlm(cameraPose));

    glDrawElements(GL_LINES, 16, GL_UNSIGNED_INT, 0);
    glBindVertexArray(0); // Unbind VAO before binding edge EBO
}