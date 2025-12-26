#include "VBEE/vbee.h"
#include <chrono>
#include <iostream>
#include <thread>

VBEESettings global_vbee_settings;

int main(int argc, char **argv) {
  float camera_x = 10;
  float camera_y = 0;
  float camera_z = 0;

  VBEE vbee(0);
  int i = 0;

  while (true) {
    vbee.Update(
        Eigen::Vector3f(camera_x, camera_y, camera_z),
        (i < 3000 || i > 5000) &&
            static_cast<bool>(
                camera_x > 0 ||
                camera_z > 0)); // Point that is seen for 3/4 of the circle

    camera_x = 10 * cos(100 * static_cast<float>(clock()) / CLOCKS_PER_SEC);
    camera_y = 10 * cos(100 * static_cast<float>(clock()) / CLOCKS_PER_SEC);
    camera_z = 10 * sin(100 * static_cast<float>(clock()) / CLOCKS_PER_SEC);

    std::cout << "Camera position: " << i << "(" << camera_x << ", " << camera_y
              << ", " << camera_z << ")" << std::endl;
    std::cout << "Estimated pExists: " << vbee.Query() << std::endl;
    std::cout << "Observability Estimate: " << vbee.GetPSeenGivenExists(Eigen::Vector3f(camera_x, camera_y, camera_z)).first << std::endl;
    std::cout << "Observability: " << vbee.GetObservability() << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    i++;
  }
}