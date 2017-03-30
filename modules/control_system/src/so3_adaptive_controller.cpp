//
// Created by xuhao on 2017/3/23.
//
#include <RapidFDM/control_system/so3_adaptive_controller.h>
#include <ctime>
#include <iomanip>
#include <L1ControlAttitude_types.h>

namespace RapidFDM {
    namespace ControlSystem {

        inline void copy_v3d_into_array(const Eigen::Vector3d v, double *arr) {
            arr[0] = v.x();
            arr[1] = v.y();
            arr[2] = v.z();
        }

        so3_adaptive_controller::so3_adaptive_controller(Aerodynamics::AircraftNode *_aircraftNode) :
                BaseController(_aircraftNode) {
            roll_sp = 0;
            pitch_sp = 0;
            //7 0.655
            //6 0.6
            //5 0.55
            //4 0.49
            //3 0.42
            init_attitude_controller(&ctrlAttitude);
//            L1ControllerUpdateParams(7.0, 0.655, 32, 7.0, 1000, &(ctrlAttitude.RollCtrl));
            L1ControllerUpdateParams(3.0, 0.42, 32, 7.0, 1000, &(ctrlAttitude.RollCtrl));
            L1ControllerUpdateParams(3.0, 0.42, 32, 7.0, 1000, &(ctrlAttitude.PitchCtrl));
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "log/so3_log_%Y_%m_%d_%H_%M_%S.mat");
            auto file = oss.str();
            printf("Creating file %s...\n\n", file.c_str());
            pmat = matOpen(file.c_str(), "w");
        }

        void so3_adaptive_controller::control_step(float deltatime) {
            static int count = 0;
            count++;
            deltatime = deltatime  / 1000;
            copy_v3d_into_array(angular_rate, sys.angular_rate);

            sys.quat[0] = quat.w();
            sys.quat[1] = quat.x();
            sys.quat[2] = quat.y();
            sys.quat[3] = quat.z();
            double u_roll = 0 ,u_pitch = 0;
            L1ControlAttitudeEuler(&ctrlAttitude,deltatime,pitch_sp,roll_sp,&sys,&u_roll,&u_pitch);

            pwm[0] = (float) u_roll;
            pwm[1] = (float) - u_pitch;
            pwm[2] = (float) throttle_sp;
            pwm[3] = (float) yaw_sp;


            aircraftNode->set_control_from_channels(pwm, 8);

            ctrl_log.push_back(ctrlAttitude.RollCtrl);
            sys_log.push_back(sys);

            if (count % 200 == 0) {
                save_data_file();
            }
        }

        void set_value_mx_array(mxArray *arr, int i, int j, double data) {
            size_t mi = mxGetM(arr);
            size_t ni = mxGetN(arr);//4
            *(mxGetPr(arr) + i + j * mi) = data;
        }

        void so3_adaptive_controller::save_data_file() {
            static int log_number = 0;
            mxArray *pa1, *pa2;
            //t 1
            //x + 6 = 7
            //gamma +1 =8
            //P + 4 = 12
            //Am + 4 = 16
            //u + 1 = 17
            //b + 2 = 19
            //kg + 1 = 20
            //init + 1 = 21

            //t 1 x 6 err 2 u 1 eta 1

            int cols = 12;
            pa1 = mxCreateDoubleMatrix(ctrl_log.size(), cols, mxREAL);
            for (int i = 0; i < ctrl_log.size(); i++) {
                const AdaptiveCtrlT &ctrlT = ctrl_log[i];
                set_value_mx_array(pa1, i, 0, ctrlT.t);
//                printf("t: %f x %f %f %f %f %f %f %d\n", ctrlT.t,
//                       ctrlT.x[0],
//                       ctrlT.x[1],
//                       ctrlT.x[2],
//                       ctrlT.x[3],
//                       ctrlT.x[4],
//                       ctrlT.x[5],
//                       ctrlT.inited
//                );

                for (int j = 0; j < 6; j++) {
                    set_value_mx_array(pa1, i, j + 1, ctrlT.x[j]);
                }
                set_value_mx_array(pa1, i, 7 , ctrlT.err[0]);
                set_value_mx_array(pa1, i, 8 , ctrlT.err[1]);
                set_value_mx_array(pa1, i, 9 , ctrlT.u);
                set_value_mx_array(pa1, i, 10 , ctrlT.eta);
                set_value_mx_array(pa1, i, 11 , ctrlT.r);
            }
            char matname[100] = {0};
            sprintf(matname, "AdaptiveCtrlT_%d", log_number);
            matPutVariable(pmat, matname, pa1);
            mxDestroyArray(pa1);
            ctrl_log.clear();
            log_number++;
        }
    }
}
