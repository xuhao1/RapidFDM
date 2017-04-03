//
// Created by xuhao on 2016/12/13.
//

#include <RapidFDM/aerodynamics/nodes/engines/easy_propeller.h>
#include <RapidFDM/common/resource_manager.h>
#include <RapidFDM/aerodynamics/geometrys/geometry_helper.h>

namespace RapidFDM {
    namespace Aerodynamics {
        float EasyPropellerNode::getJ(const ComponentData &data, const AirState &airState) const {
            double wind_speed = -get_air_velocity(data, airState).x();
            double n = get_internal_state("thrust") * max_n;
            if (wind_speed < 0.01 || n < 1)
                return 0;
            double J = wind_speed / n / D;
            return float_constrain(J, -0.2, 1);
        }

        float EasyPropellerNode::get_cq(const ComponentData &data, const AirState &airState) const {
            return Cq_spline(getJ(data, airState));
        }


        float EasyPropellerNode::get_ct(ComponentData data, AirState airState) const {
            return Ct_spline(getJ(data, airState));
        }

        double EasyPropellerNode::get_propeller_force(ComponentData data, AirState airState) const {
            double n = get_internal_state("thrust") * max_n;
            return get_ct(data, airState) * airState.rho * n * n * pow(D, 4);
        }

        double EasyPropellerNode::get_propeller_torque(ComponentData data, AirState airState) const {
            double n = get_internal_state("thrust") * max_n;
            return get_cq(data, airState) * airState.rho * n * n * pow(D, 5) * prop_dir;
        }

        EasyPropellerNode::EasyPropellerNode(const rapidjson::Value &document)
                :
                BaseEngineNode(document) {
            if (document.HasMember("geometry") && document["geometry"].IsObject()) {
                this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
            } else {
                this->geometry = new BaseGeometry();
            }
            D = fast_value(document, "D", 0.254);
            prop_dir = fast_value(document, "direction", 1);
            max_n = fast_value(document, "max_rpm", 10000.0) / 60.0;
            this->node_type = AerodynamicsNodeType::AerodynamicsEasyPropellerNode;
            std::string data_name = fast_string(document, "data_name");
            if (data_name == "") {
                data_name = "magf_7x4_2915cm_6998.txt";
            }
//            printf("Success parse propeller_node\n");
//            printf("Name %s type: %s geometry %s using data %s\n", this->name.c_str(), this->get_type_str().c_str(),
//                   geometry->get_type().c_str(),
//                   data_name.c_str()
//            );
            this->freq_cut = fast_value(document, "freq_cut", 5);
            std::string data_root = RapidFDM::Common::get_data_path();
            std::ifstream ifs(data_root + "/propellers/" + data_name + ".txt");
            std::vector<double> jdata, ctdata, cqdata;
            if (!ifs.is_open()) {
                std::cerr << "Error while open prop data file on "<< data_root + "/propellers/" + data_name + ".txt\n";
                std::abort();
            } else {
                std::string tmp;
                std::getline(ifs, tmp);
//                printf("%s\n", tmp.c_str());
                while (!ifs.eof()) {
                    double j, ct, cq, eta;
                    ifs >> j >> ct >> cq >> eta;
                    cq = cq / M_PI;
//                    printf("%f %f %f %f\n",
//                           j, ct, cq, eta
//                    );
                    jdata.push_back(j);
                    ctdata.push_back(ct);
                    cqdata.push_back(cq);
                }
                jdata.pop_back();
                ctdata.pop_back();
                cqdata.pop_back();
                Ct_spline.set_points(jdata, ctdata);

                Cq_spline.set_points(jdata, cqdata);
            }
            this->control_axis[getUniqueID() + "/thrust"] = 0;
            this->internal_states[getUniqueID() + "/thrust"] = 0;

        }

        Eigen::Vector3d EasyPropellerNode::get_engine_force(ComponentData data, AirState airState) const {
//                Force to negative x axis
                return Eigen::Vector3d(-get_propeller_force(data, airState), 0, 0);
        }

        Eigen::Vector3d EasyPropellerNode::get_engine_torque(ComponentData data, AirState airState) const {
//                Torque at negative x axis
                return Eigen::Vector3d(get_propeller_torque(data, airState), 0, 0);
        }
    }
}
