#include "VonMiserSphere.h"

VonMiserSphere::VonMiserSphere(int n_sides) : n_sides(n_sides)
{
    GLuint VBO, EBO = 0;

    // Generate the VAO for the Von Mises sphere
    glGenVertexArrays(1, &VAO);
    glBindVertexArray(VAO);

    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    std::vector<float> vertices;
    vertices.insert(vertices.end(), {0.0f, 0.0f, 0.0f}); // Center vertex
    for (int i = 0; i < n_sides; ++i)
    {
        vertices.push_back(cos(2.0f * M_PI * i / n_sides)); // x
        vertices.push_back(sin(2.0f * M_PI * i / n_sides)); // y
        vertices.push_back(0.0f);                           // z
    }

    // Create the Von Mises sphere using the IcosahedronBackend
    // IcosahedronBackend icosahedronBackend;
    // icosahedronBackend.createIcosahedron(1.0f, 3); // Radius 1.0, 3 subdivisions

    // Bind the vertex buffer and element buffer
    // glBindBuffer(GL_ARRAY_BUFFER, icosahedronBackend.getVertexBuffer());
    // glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, icosahedronBackend.getElementBuffer());

    // Set vertex attribute pointers
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0); // Unbind the VAO
}