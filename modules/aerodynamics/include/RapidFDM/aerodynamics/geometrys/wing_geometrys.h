//
// Created by xuhao on 2016/5/5.
//

#ifndef RAPIDFDM_AERODYNAMICS_WING_GEOMETRYS_H
#define RAPIDFDM_AERODYNAMICS_WING_GEOMETRYS_H

#include <RapidFDM/aerodynamics/geometrys/base_geometry.h>
#include <utils.h>

namespace RapidFDM {
    namespace Aerodynamics {
        class WingGeometry : public BaseGeometry {
        public:
            virtual float getLift(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }

            virtual float getDrag(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }

            virtual float getSide(ComponentData state, AirState airState) {
                std::cerr << "Code not wrote" << std::endl;
                abort();
                return 0;
            }
            //!Build a wing
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
            struct {
                float b_2;
                float Mac;
                bool nonSideAttach = true;
                float TarperRatio = 1;
                float MidChordSweep = 0;
                float maxdeflect = 15;
                float ctrlSurfFrac = 0.0;
                bool enableControl = false;
                int wingPart = 3;

            } params;

            WingGeometry(rapidjson::Value &v) {
                params.b_2 = fast_value(v, "b_2", 0);
                params.Mac = fast_value(v, "Mac", 0);
                params.nonSideAttach = fast_value(v, "nonSideAttch", 1);
                params.TarperRatio = fast_value(v, "TaperRatio", 1);
                params.MidChordSweep = fast_value(v, "MidChordSweep", 0);
                params.maxdeflect = fast_value(v, "maxdeflect", 15);
                params.enableControl = fast_value(v, "enableControl", 0) == 1;
                if (params.enableControl)
                    params.ctrlSurfFrac = fast_value(v, "ctrlSurfFrac", 0.2);
                params.wingPart = fast_value(v, "wingPart", 3);
                this->type = "WingGeometry";
            }

            virtual void brief() {
                if (this->params.enableControl)
                    printf("WindControlable\n");
                else
                    printf("WingDiscontrolable\n");
                printf("b_2 %f\n", params.b_2);
                if (params.wingPart == 0)
                    printf("wing on left side\n");
                else if (params.wingPart == 1)
                    printf("Wing on right side\n");
                else
                    printf("Wing on both side\n");

            }
        };
    }
}

#endif //RAPIDFDM_WING_GEOMETRYS_H
