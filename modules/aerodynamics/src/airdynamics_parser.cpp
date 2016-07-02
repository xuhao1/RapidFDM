#include <RapidFDM/aerodynamics/airdynamics_parser.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        parser::parser(std::string path)
        {
            //TODO:
            //Instancing nodes
            this->nodes = NodeHelper::scan_node_folder(path + "/nodes/");

            for (auto pair : nodes) {
                printf("Nodes name: %s, id : %s, type : %s\n",
                       pair.second->getName().c_str(),
                       pair.first.c_str(),
                       pair.second->get_type_str().c_str());
            }
            this->joints = JointHelper::scan_joint_folder(path + "/joints/", this->nodes);
            rapidjson::Document d;
            d.Parse(get_string_from_file(std::string(path + "/main.json")).c_str());
            assert(d.IsObject());

            std::string root_id = fast_string(d, "root");
            this->aircraftNode = static_cast<AircraftNode *> (this->nodes[root_id]);
            if (this->aircraftNode == nullptr) {
                std::cerr << "Error while parse aircraft! No aircraft node " << root_id << "found!!!" << std::endl;
                std::abort();
            }
            else {
                this->aircraftNode->init_after_construct(this->nodes,this->joints);
                std::cout << "Successful parse aircraft :" << this->aircraftNode->getName() << "\n" << std::endl;
            }

        }
    }
}
