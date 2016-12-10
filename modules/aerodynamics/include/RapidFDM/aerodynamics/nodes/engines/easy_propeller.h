//
// Created by Hao Xu on 16/6/27.
//

#ifndef RAPIDFDM_EASY_PROPELLER_H
#define RAPIDFDM_EASY_PROPELLER_H

#include <RapidFDM/aerodynamics/nodes/engines/base_engine.h>
#include <spline.h>
#include <fstream>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        //! This node stand for a electrical_propeller, no need battery and no acceleration time
        //  this propeller perfermance spline is independce beyond D and H
        class EasyPropellerNode : public BaseEngineNode
        {
			int prop_dir;
        protected:
            //Ct = A_ct * J + B_ct;
            //T = rho * n^2 * D^4 * Ct
            //Cq = 0.05 / 2pi
            //Q = rho * n^2 * D^5 * Cq
            
//            double A_ct = -0.10;
//            double B_ct = 0.09;
//            double C_q = 0.0;
            double D;
            double max_n;
        
            //Ct alone j
            //Cq alone j
            tk::spline Ct_spline,Cq_spline;
            
            virtual float getJ(const ComponentData & data,const AirState & airState) const {
                double wind_speed = -get_air_velocity(data, airState).x();
                auto pair = internal_states.find("thrust");
                double n = pair->second * max_n;
                if (wind_speed < 0.01 || n < 1)
                    return 0;
                double J =  wind_speed / n / D;
                return float_constrain(J,0,1);
            }

            virtual float get_cq(const ComponentData & data,const  AirState & airState) const
            {
                return Cq_spline(getJ(data,airState));
            }

            virtual float get_ct(ComponentData data, AirState airState) const
            {
                return Ct_spline(getJ(data,airState));
            }

            virtual float get_propeller_force(ComponentData data, AirState airState) const
            {
                auto pair = internal_states.find("thrust");
                double n = pair->second * max_n;
                return get_ct(data, airState) * airState.rho * n * n * pow(D, 4);
            }
			

            virtual float get_propeller_torque(ComponentData data, AirState airState) const
            {
                auto pair = internal_states.find("thrust");
                double n = pair->second * max_n;
                return get_cq(data, airState) * airState.rho * n * n * pow(D, 5) * prop_dir;
            }

        public:

            virtual Eigen::Vector3d get_engine_force(ComponentData data, AirState airState) const override
            {
                //Force to negative x axis
                return Eigen::Vector3d(-get_propeller_force(data, airState), 0, 0);
            }

            virtual Eigen::Vector3d get_engine_torque(ComponentData data, AirState airState) const override
            {
                //Torque at negative x axis
                return Eigen::Vector3d(get_propeller_torque(data, airState), 0, 0);
            }

            virtual BaseNode *instance() override
            {
                BaseEngineNode *node = new BaseEngineNode;
                memcpy(node, this, sizeof(BaseEngineNode));
                node->geometry = this->geometry->instance();
                return node;
            }


            EasyPropellerNode(const rapidjson::Value &document) :
                    BaseEngineNode(document)
            {
                if (document.HasMember("geometry") && document["geometry"].IsObject()) {
                    this->geometry = GeometryHelper::create_geometry_from_json(document["geometry"]);
                }
                else {
                    this->geometry = new BaseGeometry();
                }
                D = fast_value(document, "D", 0.254);
                prop_dir = fast_value(document, "direction", 1);
                max_n = fast_value(document, "max_rpm", 10000.0) / 60.0;
                this->node_type = AerodynamicsNodeType::AerodynamicsEasyPropellerNode;
                std::string data_name = fast_string(document,"data_name");
                if(data_name=="")
                {
                    data_name = "magf_7x4_2915cm_6998.txt";
                }
                printf("Success parse propeller_node\n");
                printf("Name %s type: %s geometry %s using data %s\n", this->name.c_str(), this->get_type_str().c_str(),
                       geometry->get_type().c_str(),
                       data_name.c_str()
                );
#ifdef __linux__ 
				std::cerr << "Code not wrote" << std::endl;
				std::abort();
#elif _WIN32
				std::string data_root = "c:/RapidFDM/data";
#else
				std::string data_root = "/Users/xuhao/Develop/FixedwingProj/RapidFDM/data";
#endif
                this->freq_cut = fast_value(document,"freq_cut",5);
                std::ifstream ifs(data_root+"/propellers/" + data_name + ".txt");
                printf("parsing propeller data\n");
                std::vector<double> jdata,ctdata,cqdata;
                if (!ifs.is_open())
                {
                    std::cerr << "Error while open prop data file" ;
					std::abort();
                }
                else {
                    std::string tmp;
                    std::getline(ifs,tmp);
                    printf("%s\n",tmp.c_str());
                    while (!ifs.eof()) {
                        double j, ct, cq, eta;
                        ifs >> j >> ct >> cq >> eta;
                        cq = cq / M_PI;
                        printf("%f %f %f %f\n",
                               j, ct, cq, eta
                        );
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
                this->control_axis["thrust"] = 0;
                this->internal_states["thrust"] = 0;

            }
        };
    }
}
#endif //RAPIDFDM_ELECTRICAL_PROPELLER_H
