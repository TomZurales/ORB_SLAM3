#pragma once

#include <Eigen/Core>
#include <vector>
#include <map>
#include <string>
#include <iostream>

#define ICOS_SHORT sqrt(2 / (5 + sqrt(5)))
#define ICOS_LONG ((1 + sqrt(5)) / 2) * ICOS_SHORT

class Icosahedron
{
private:
    std::map<std::string, std::vector<float>> valueBuffers;

    int lastFace = -1;
    std::vector<Eigen::Vector3f> prevObservations;

    // Overall probability that this point exists
    float probExists = 0.95f;

public:
    Icosahedron();
    ~Icosahedron() = default;

    int getNearestFace(Eigen::Vector3f in);

    void createValueBuffer(const std::string &name, float initialValue = 0.0f);
    float getValue(const std::string &name, int index);
    void setValue(const std::string &name, int index, float value);
    float getMinValue(const std::string &name);
    float getMaxValue(const std::string &name);
    // Gets a value normalized to the range [0, 1]
    float getNormalizedValue(const std::string &name, int index);
    float getPDFValue(const std::string &name, int index);

    int getLastFace() const { return lastFace; }
    void setLastFace(int face) { lastFace = face; }
    const std::vector<Eigen::Vector3f> &getPrevObservations() const { return prevObservations; }
    void addPrevObservation(const Eigen::Vector3f &observation) { prevObservations.push_back(observation); }
    void clearPrevObservations() { prevObservations.clear(); }
    float getProbExists() const { return probExists; }
    void setProbExists(float probExists) { this->probExists = probExists; }

    std::vector<Eigen::Vector3f> vertices = {
        Eigen::Vector3f(0, ICOS_LONG, ICOS_SHORT).normalized(),   // 1
        Eigen::Vector3f(0, ICOS_LONG, -ICOS_SHORT).normalized(),  // 2
        Eigen::Vector3f(ICOS_LONG, ICOS_SHORT, 0).normalized(),   // 3
        Eigen::Vector3f(ICOS_SHORT, 0, -ICOS_LONG).normalized(),  // 4
        Eigen::Vector3f(-ICOS_SHORT, 0, -ICOS_LONG).normalized(), // 5
        Eigen::Vector3f(-ICOS_LONG, ICOS_SHORT, 0).normalized(),  // 6
        Eigen::Vector3f(-ICOS_SHORT, 0, ICOS_LONG).normalized(),  // 7
        Eigen::Vector3f(ICOS_SHORT, 0, ICOS_LONG).normalized(),   // 8
        Eigen::Vector3f(ICOS_LONG, -ICOS_SHORT, 0).normalized(),  // 9
        Eigen::Vector3f(0, -ICOS_LONG, -ICOS_SHORT).normalized(), // 10
        Eigen::Vector3f(-ICOS_LONG, -ICOS_SHORT, 0).normalized(), // 11
        Eigen::Vector3f(0, -ICOS_LONG, ICOS_SHORT).normalized()   // 12
    };

    std::vector<std::vector<int>>
        faces = {
            {0, 1, 2},   // 1
            {2, 1, 3},   // 2
            {3, 1, 4},   // 3
            {4, 1, 5},   // 4
            {5, 1, 0},   // 5
            {5, 0, 6},   // 6
            {6, 0, 7},   // 7
            {0, 2, 7},   // 8
            {7, 2, 8},   // 9
            {2, 3, 8},   // 10
            {8, 3, 9},   // 11
            {3, 4, 9},   // 12
            {9, 4, 10},  // 13
            {4, 5, 10},  // 14
            {10, 5, 6},  // 15
            {10, 6, 11}, // 16
            {6, 7, 11},  // 17
            {7, 8, 11},  // 18
            {8, 9, 11},  // 19
            {9, 10, 11}  // 20
    };
    std::vector<Eigen::Vector3f> face_centers;
    std::vector<std::vector<int>> edges = {
        {0, 1},
        {1, 2},
        {0, 2},
        {1, 3},
        {2, 3},
        {1, 4},
        {3, 4},
        {1, 5},
        {4, 5},
        {5, 0},
        {0, 7},
        {2, 7},
        {2, 8},
        {7, 8},
        {3, 8},
        {3, 9},
        {8, 9},
        {4, 9},
        {4, 10},
        {9, 10},
        {5, 10},
        {5, 6},
        {6, 10},
        {0, 6},
        {6, 7},
        {7, 11},
        {8, 11},
        {9, 11},
        {10, 11},
        {6, 11}};

    std::vector<Eigen::Vector2f> vertices_2d =
        {
            Eigen::Vector2f(0.5, 0),             // 1
            Eigen::Vector2f(1.5, 0),             // 2
            Eigen::Vector2f(2.5, 0),             // 3
            Eigen::Vector2f(3.5, 0),             // 4
            Eigen::Vector2f(4.5, 0),             // 5
            Eigen::Vector2f(0, sqrt(3) / 2),     // 6
            Eigen::Vector2f(1, sqrt(3) / 2),     // 7
            Eigen::Vector2f(2, sqrt(3) / 2),     // 8
            Eigen::Vector2f(3, sqrt(3) / 2),     // 9
            Eigen::Vector2f(4, sqrt(3) / 2),     // 10
            Eigen::Vector2f(5, sqrt(3) / 2),     // 11
            Eigen::Vector2f(0.5, sqrt(3)),       // 12
            Eigen::Vector2f(1.5, sqrt(3)),       // 13
            Eigen::Vector2f(2.5, sqrt(3)),       // 14
            Eigen::Vector2f(3.5, sqrt(3)),       // 15
            Eigen::Vector2f(4.5, sqrt(3)),       // 16
            Eigen::Vector2f(5.5, sqrt(3)),       // 17
            Eigen::Vector2f(1, 3 * sqrt(3) / 2), // 18
            Eigen::Vector2f(2, 3 * sqrt(3) / 2), // 19
            Eigen::Vector2f(3, 3 * sqrt(3) / 2), // 20
            Eigen::Vector2f(4, 3 * sqrt(3) / 2), // 21
            Eigen::Vector2f(5, 3 * sqrt(3) / 2)  // 22
    };

    std::vector<std::vector<int>> faces_2d = {
        {0, 5, 6},    // 1
        {1, 6, 7},    // 2
        {2, 7, 8},    // 3
        {3, 8, 9},    // 4
        {4, 9, 10},   // 5
        {9, 10, 15},  // 6
        {10, 15, 16}, // 7
        {5, 6, 11},   // 8
        {6, 11, 12},  // 9
        {6, 7, 12},   // 10
        {7, 12, 13},  // 11
        {7, 8, 13},   // 12
        {13, 8, 14},  // 13
        {8, 9, 14},   // 14
        {14, 9, 15},  // 15
        {14, 15, 20}, // 16
        {15, 16, 21}, // 17
        {11, 12, 17}, // 18
        {12, 13, 18}, // 19
        {13, 14, 19}  // 20
    };
};