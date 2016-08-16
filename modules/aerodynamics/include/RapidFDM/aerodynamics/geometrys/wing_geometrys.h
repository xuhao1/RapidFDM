//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AERODYNAMICS_WING_GEOMETRYS_H
#define RAPIDFDM_AERODYNAMICS_WING_GEOMETRYS_H

#include <RapidFDM/aerodynamics/geometrys/base_geometry.h>
#include <RapidFDM/utils.h>

namespace RapidFDM
{
    namespace Aerodynamics
    {
        class WingGeometry : public BaseGeometry
        {
        protected:
        public:
            virtual Eigen::Vector3d get_aerodynamics_torque(ComponentData state,AirState airState) const override ;
            virtual Eigen::Vector3d get_aerodynamics_force(ComponentData state,AirState airState) const override ;

            /*flow KAR parameter
             * MODULE
                b_2 = 0.5               //distance from wing root to tip; semi-span
                MAC = 0.5               //Mean Aerodynamic Chord
                nonSideAttach = 0           //0 for canard-like / normal wing pieces, 1 for ctrlsurfaces attached to the back of other wing parts
                TaperRatio = 0.7            //Ratio of tip chord to root chord generally < 1, must be > 0
                MidChordSweep = 25          //Sweep angle in degrees; measured down the center of the span / midchord position
                maxdeflect = 15             //Default maximum deflection value; only used by FARControlableSurface
                controlSurfacePivot = 1, 0, 0;      //Local vector that obj_ctrlSrf pivots about; defaults to 1, 0, 0 (right)
                ctrlSurfFrac = 0.2          //Value from 0-1, percentage of the part that is a flap; only used by FARControlableSurface
                wing part: 0 control wing on negative part of y-axis, 1 on positive part, 3 both
            */
            struct
            {
                float b_2;
                float Mac;
                bool nonSideAttach = true;
                float TarperRatio = 1;
                float MidChordSweep = 0;
                float maxdeflect = 15;
                float ctrlSurfFrac = 0.0;
                bool enableControl = false;
                int wingPart = 3;
                double deflectAngle = 0;
                double root_chord_length = 0;
                double taper_chord_length = 0;
                double flap_coeff = 1;
                double cl0;
                double cl_by_deg;
                double cd0;
                double cd_by_deg2;
                double stall_angle;

            } params;
            double aera = 0;

            WingGeometry(const rapidjson::Value &v);

            virtual void brief();

        };
    }
}

#endif //RAPIDFDM_WING_GEOMETRYS_H
