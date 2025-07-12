#include "IcosModel.h"

IcosModel::IcosModel()
{
    unsigned int icosModelVBO;

    glGenVertexArrays(1, &icosVAO);
    glGenBuffers(1, &icosModelVBO);
    glGenBuffers(1, &icosEBO);

    glBindVertexArray(icosVAO);

    Icosahedron icos;
    std::vector<float> pointData;
    for (const auto &point : icos.vertices)
    {
        pointData.push_back(point.x());
        pointData.push_back(point.y());
        pointData.push_back(point.z());
    }
    std::vector<unsigned int> indices;
    for (const auto &face : icos.faces)
    {
        indices.push_back(face[0]);
        indices.push_back(face[1]);
        indices.push_back(face[2]);
    }
    glBindBuffer(GL_ARRAY_BUFFER, icosModelVBO);
    glBufferData(GL_ARRAY_BUFFER, pointData.size() * sizeof(float), pointData.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, icosEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(unsigned int), indices.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
}

void IcosModel::draw(PPEPointInterface *point, Icosahedron *icos) const
{
    if (!icos)
        return;

    // --- Draw faces with heatmap colors ---
    ShaderManager *sm = ShaderManager::getInstance();
    sm->useShader("heatmap_3d");
    sm->setModelMatrix(glm::translate(glm::mat4(1.0f), glm::vec3(point->getPosition().x(), point->getPosition().y(), point->getPosition().z())));

    std::vector<float> intensities;
    for (int i = 0; i < 20; i++)
    {
        intensities.push_back(icos->getPDFValue("probSeen", i));
    }
    sm->setFaceValues(intensities);

    glBindVertexArray(icosVAO);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glCullFace(GL_BACK);
    glFrontFace(GL_CCW);
    glDrawElements(GL_TRIANGLES, icos->faces.size() * 3, GL_UNSIGNED_INT, 0);

    // // --- Draw wireframe edges ---
    sm->useShader("solid_color_3d");
    sm->setColor(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f)); // Black color

    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glDisable(GL_CULL_FACE);
    glDrawElements(GL_TRIANGLES, icos->faces.size() * 3, GL_UNSIGNED_INT, 0);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Reset to fill mode

    glBindVertexArray(0);
}