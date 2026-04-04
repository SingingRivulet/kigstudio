
#include "kigstudio/utils/dbvt3d.h"

int main() {
    sinriv::kigstudio::dbvt3d<float, int> dbvt;
    int id[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    dbvt.add(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(0, 0, 0),
             sinriv::kigstudio::dbvt3d<float, int>::vec3_n(1, 1, 1), &id[0]);
    dbvt.add(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(0.5, 0.5, 0.5),
             sinriv::kigstudio::dbvt3d<float, int>::vec3_n(1.5, 1.5, 1.5), &id[1]);

    std::cout << "search:(0.3, 0.3, 0.3)" << std::endl;
    dbvt.fetchByPoint(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(0.3, 0.3, 0.3), [&](auto node){
        std::cout << "id: " << *(int*)node->data << std::endl;
    });

    std::cout << "search:(0.5, 0.5, 0.5)" << std::endl;
    dbvt.fetchByPoint(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(0.5, 0.5, 0.5), [&](auto node){
        std::cout << "id: " << *(int*)node->data << std::endl;
    });

    std::cout << "search:(1.2, 1.2, 1.2)" << std::endl;
    dbvt.fetchByPoint(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(1.2, 1.2, 1.2), [&](auto node){
        std::cout << "id: " << *(int*)node->data << std::endl;
    });
    
    std::cout << "search:(1.2, 1.2, 1.7)" << std::endl;
    dbvt.fetchByPoint(sinriv::kigstudio::dbvt3d<float, int>::vec3_n(1.2, 1.2, 1.7), [&](auto node){
        std::cout << "id: " << *(int*)node->data << std::endl;
    });

    std::cout << "ray test:(0, 0, 0) -> (1, 1, 1)" << std::endl;
    dbvt.rayTest(
        sinriv::kigstudio::ray<float>(
            sinriv::kigstudio::dbvt3d<float, int>::vec3_n(0, 0, 0),
            sinriv::kigstudio::dbvt3d<float, int>::vec3_n(1, 1, 1)
        ),
        [&](auto node){
            std::cout << "id: " << *node->data << std::endl;
        }
    );

    std::cout << "success" << std::endl;
    return 0;
}