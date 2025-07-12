#include "PointCloud.h"
#include <iostream>

PointCloud::PointCloud()
{
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

void PointCloud::draw(std::vector<PPEPointInterface *> points) const
{
    if (points.empty())
        return;

    std::vector<float> outOfFrustumData;
    std::vector<float> hiddenData;
    std::vector<float> visibleData;
    for (const auto &point : points)
    {
        Eigen::Vector3f position = point->getPosition();

        if (!point->isInView())
        {
            outOfFrustumData.push_back(position.x());
            outOfFrustumData.push_back(position.y());
            outOfFrustumData.push_back(position.z());
        }
        else if (!point->isVisible())
        {
            hiddenData.push_back(position.x());
            hiddenData.push_back(position.y());
            hiddenData.push_back(position.z());
        }
        else
        {
            visibleData.push_back(position.x());
            visibleData.push_back(position.y());
            visibleData.push_back(position.z());
        }
    }

    glBindVertexArray(VAO);

    ShaderManager *sm = ShaderManager::getInstance();
    sm->useShader("solid_color_3d");
    sm->setModelMatrix(modelPose);
    sm->setColor(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Set color to black

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, outOfFrustumData.size() * sizeof(GLfloat), outOfFrustumData.data(), GL_STREAM_DRAW);

    glEnable(GL_DEPTH_TEST);
    glPointSize(2.0f);
    glDrawArrays(GL_POINTS, 0, outOfFrustumData.size() / 3);

    sm->setColor(glm::vec4(1.0f, 0.0f, 0.0f, 1.0f)); // Set color to red
    glBufferData(GL_ARRAY_BUFFER, hiddenData.size() * sizeof(GLfloat), hiddenData.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_POINTS, 0, hiddenData.size() / 3);

    sm->setColor(glm::vec4(0.0f, 1.0f, 0.0f, 1.0f)); // Set color to green
    glBufferData(GL_ARRAY_BUFFER, visibleData.size() * sizeof(GLfloat), visibleData.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_POINTS, 0, visibleData.size() / 3);

    glBindVertexArray(0);
}