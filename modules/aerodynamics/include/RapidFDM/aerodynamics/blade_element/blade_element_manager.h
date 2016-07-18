//
// Created by Hao Xu on 16/7/18.
//

#ifndef RAPIDFDM_BLADE_MANAGER_H
#define RAPIDFDM_BLADE_MANAGER_H

#include <RapidFDM/aerodynamics/blade_element/base_blade_element.h>
#include <RapidFDM/aerodynamics/blade_element/blade_element_manager.h>
#include <vector>

namespace RapidFDM
{
    namespace Aerodynamics{
        class BladeElementManager {
            std::map < BaseBladeElement * ,AirState * > blade_elements;
        public:
            BladeElementManager() {

            }
            void add_blade_element(BaseBladeElement * blade);

            void add_blade_elements(std::vector<BaseBladeElement *> blades);

            AirState get_blade_element_airstate(BaseBladeElement * element);

            void calculate_washes(AirState airState);
        };
    }
}

#endif //RAPIDFDM_BLADE_MANAGER_H
