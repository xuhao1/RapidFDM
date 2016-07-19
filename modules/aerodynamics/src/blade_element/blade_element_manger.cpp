//
// Created by Hao Xu on 16/7/18.
//


#include <RapidFDM/aerodynamics/blade_element/blade_element_manager.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        void BladeElementManager::calculate_washes(AirState airState, double deltatime)
        {
            //TODO:
            for (auto pair : blade_elements) {
                pair.second->rho = airState.rho;
                pair.second->ground_air_speed = airState.ground_air_speed;
            }
        }

        void BladeElementManager::add_blade_element(BaseBladeElement *blade)
        {
            blade_elements[blade] = new AirState;
            blade_elements[blade]->ground_air_speed;
        }

        AirState BladeElementManager::get_blade_element_airstate(BaseBladeElement *element)
        {
            return *blade_elements[element];
        }

        void BladeElementManager::add_blade_elements(std::vector<BaseBladeElement *> blades)
        {
            for (auto blade : blades) {
                add_blade_element(blade);
            }
        }

        rapidjson::Document *BladeElementManager::get_blades_information()
        {
            rapidjson::Document *d = new rapidjson::Document;
            d->SetArray();

            for (auto pair:blade_elements) {
                rapidjson::Value object(rapidjson::kObjectType);
                BaseBladeElement *blade = pair.first;
                AirState *second = pair.second;
                Eigen::Vector3d force = blade->get_aerodynamics_force(
                        blade->get_component_data_from_geometry(),
                        *second
                );
                force = blade->get_ground_transform().linear() * force;
                add_vector(object, force, *d, "force");
                Eigen::Vector3d location = (Eigen::Vector3d)blade->get_body_transform().translation();
                add_vector(object,location,*d,"location");
                d->PushBack(object,d->GetAllocator());
            }
            return d;

        }
    }
}