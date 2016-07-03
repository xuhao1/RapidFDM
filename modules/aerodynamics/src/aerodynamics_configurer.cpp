//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/aerodynamics/aerodynamics_configurer.h>
using namespace RapidFDM::Utils;
namespace RapidFDM
{
    namespace Aerodynamics {
        aerodynamics_configurer::aerodynamics_configurer(std::string root_path)
        {
            this->root_path = root_path;
        }
        void aerodynamics_configurer::chroot_folder(std::string path)
        {
            this->root_path = path;
        }

        std::vector<std::string> aerodynamics_configurer::list_model()
        {
            return  get_file_list(root_path);
        }

        void aerodynamics_configurer::load_model(std::string name)
        {
            if (parser1!= nullptr)
            {
                delete parser1;
                parser1 = nullptr;
            }
            parser1 = new parser(root_path + name);
        }

        void aerodynamics_configurer::save_model(std::string name)
        {
        }
        void aerodynamics_configurer::update_model(rapidjson::Value &v)
        {

        }

    }
}