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
            
            double D;
            double max_n;
        
            //Ct alone j
            //Cq alone j
            tk::spline Ct_spline,Cq_spline;
            
            virtual float getJ(const ComponentData & data,const AirState & airState) const;

            virtual float get_cq(const ComponentData & data,const  AirState & airState) const;

            virtual float get_ct(ComponentData data, AirState airState) const;

            virtual double get_propeller_force(ComponentData data, AirState airState) const;

            virtual double get_propeller_torque(ComponentData data, AirState airState) const;

        public:

            virtual Eigen::Vector3d get_engine_force(ComponentData data, AirState airState) const override;

            virtual Eigen::Vector3d get_engine_torque(ComponentData data, AirState airState) const override;

            virtual BaseNode *instance() override
            {
                return nullptr;
            }


            EasyPropellerNode(const rapidjson::Value &document);

        };
    }
}
#endif //RAPIDFDM_ELECTRICAL_PROPELLER_H
