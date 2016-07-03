//
// Created by Hao Xu on 16/6/11.
//

#include <RapidFDM/aerodynamics/aerodynamics_configurer.h>

using namespace RapidFDM::Utils;
namespace RapidFDM
{
    namespace Aerodynamics
    {
        aerodynamics_configurer::aerodynamics_configurer(std::string root_path)
        {
            this->root_path = root_path;
            init_query_functions();
        }

        void aerodynamics_configurer::chroot_folder(std::string path)
        {
            this->root_path = path;
        }

        std::vector<std::string> aerodynamics_configurer::list_model()
        {
            return get_file_list(root_path);
        }

        void aerodynamics_configurer::load_model(std::string name)
        {
            if (parser1 != nullptr) {
                delete parser1;
                parser1 = nullptr;
            }
            parser1 = new parser(root_path + name);
            this->aircraftNode = parser1->get_aircraft_node();
        }

        void aerodynamics_configurer::save_model(std::string name)
        {
        }

        void aerodynamics_configurer::update_model(const rapidjson::Value &v)
        {

        }

        rapidjson::Document * aerodynamics_configurer::query_model(const rapidjson::Value &v)
        {
            std::string query_code = v.GetString();
            if (aircraftNode != nullptr && query_functions.find(query_code) != query_functions.end()) {
                return query_functions[query_code](v);
            }
            else
            {
                rapidjson::Document * d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode","response_query",d->GetAllocator());
                rapidjson::Value str(rapidjson::kStringType);
                str.SetString(query_code.c_str(),d->GetAllocator());
                d->AddMember("type",str,d->GetAllocator());
                d->AddMember("data","no object loaded",d->GetAllocator());
            }
        }

        void aerodynamics_configurer::init_query_functions()
        {
            query_functions["get_total_mass_inertial"] = [&](const rapidjson::Value &v) {
                rapidjson::Document * d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode","response_query",d->GetAllocator());
                d->AddMember("type","get_total_mass_inertial",d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);

                add_value(value, aircraftNode->get_total_mass(), *d, "total_mass");
                add_vector(value, aircraftNode->get_total_inertial(),*d, "total_inertial");

                d->AddMember("data",value,d->GetAllocator());
                return d;
            };
            query_functions["get_total_forces_torques"] = [&](const rapidjson::Value &v) {
                rapidjson::Document * d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode","response_query",d->GetAllocator());
                d->AddMember("type","get_total_forces_torques",d->GetAllocator());
                rapidjson::Value value(rapidjson::kObjectType);

                add_vector(value, aircraftNode->get_total_force(), *d, "total_force");
                add_vector(value, aircraftNode->get_total_torque(), *d, "total_torue");

                add_vector(value, aircraftNode->get_total_aerodynamics_force(), *d, "total_airdynamics_force");
                add_vector(value, aircraftNode->get_total_aerodynamics_torque(), *d, "total_airdynamics_torque");

                add_vector(value, aircraftNode->get_total_engine_torque(), *d, "total_engine_torque");
                add_vector(value, aircraftNode->get_total_engine_force(), *d, "total_engine_force");

                d->AddMember("data",value,d->GetAllocator());
                return d;
            };
            query_functions["get_internal_states_list"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode","response_query",d->GetAllocator());
                d->AddMember("type","get_internal_states_list",d->GetAllocator());
                rapidjson::Value value(rapidjson::kArrayType);

                for (auto pair : aircraftNode->get_internal_states())
                {
                    rapidjson::Value v(rapidjson::kObjectType);
                    rapidjson::Value name(rapidjson::kStringType);
                    name.SetString(pair.first.c_str(),d->GetAllocator());
                    v.AddMember(name,pair.second,d->GetAllocator());
                    value.PushBack(v,d->GetAllocator());
                }

                d->AddMember("data",value,d->GetAllocator());
                return d;
            };
             query_functions["get_control_axis_list"] = [&](const rapidjson::Value &v) {
                rapidjson::Document *d = new rapidjson::Document;
                d->SetObject();
                d->AddMember("opcode","response_query",d->GetAllocator());
                d->AddMember("type","get_control_axis_list",d->GetAllocator());
                rapidjson::Value value(rapidjson::kArrayType);

                for (auto pair : aircraftNode->get_control_axis())
                {
                    rapidjson::Value v(rapidjson::kObjectType);
                    rapidjson::Value name(rapidjson::kStringType);
                    name.SetString(pair.first.c_str(),d->GetAllocator());
                    v.AddMember(name,pair.second,d->GetAllocator());
                    value.PushBack(v,d->GetAllocator());
                }

                d->AddMember("data",value,d->GetAllocator());
                return d;
            };
        }
    }
}