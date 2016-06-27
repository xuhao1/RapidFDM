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
            this->joints = JointHelper::scan_joint_folder(path + "/joints/",this->nodes);
            rapidjson::Document d;
            d.Parse(std::string(path + "/main.json").c_str());

            std::string root_id = fast_string(d,"root");
            this->aircraftNode = static_cast<AircraftNode *> (this->nodes[root_id]);
            if (this->aircraftNode == nullptr)
            {
                std::cerr << "Error while parse aircraft! No aircraft node " << root_id << "found!!!" << std::endl;
                std::abort();
            }
        }
    }
}
