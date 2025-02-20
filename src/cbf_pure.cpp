#include "utils.h"
#include "Swarm.hpp"

#include <fstream>
#include <ctime>

#include "ament_index_cpp/get_package_share_directory.hpp"

int main() {
    clock_t start = clock();
    
    std::string pkg_name = "cbf-ros2";
    std::string share_path = ament_index_cpp::get_package_share_directory(pkg_name);
    std::string config_path = share_path + "/config/config.json";

    json settings = json::parse(std::ifstream(config_path));

    std::cout << settings << std::endl;

    Swarm(settings).run();

    clock_t finish = clock();
    double duration = (double) (finish - start) / CLOCKS_PER_SEC;
    printf("%.4lf seconds passed!\n", duration);

}