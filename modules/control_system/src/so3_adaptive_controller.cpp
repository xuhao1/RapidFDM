//
// Created by xuhao on 2017/3/23.
//
#include <RapidFDM/control_system/so3_adaptive_controller.h>
#include <ctime>
#include <iomanip>
#include <L1ControlAttitude_types.h>
#include <L1ControlAttitude.h>

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
            double lead_fc = 2;
            double lead_alpha = 1; //multirotor
//            double lead_fc = 0.8;
//            double lead_alpha = 1;
            double lag_fc = 7;
            double lag_alpha = 15;
//            double lag_fc = 7;
//            double lag_alpha = 15;
            L1ControllerUpdateParams(&(ctrlAttitude.RollCtrl), 7.0, 1.0, 32, 1000, lag_fc, lag_alpha, lead_fc,
                                     lead_alpha);
            L1ControllerUpdateParams(&(ctrlAttitude.PitchCtrl), 7.0,1.0, 32, 1000, lag_fc, lag_alpha, lead_fc,
                                     lead_alpha);
            L1ControllerUpdateParams(&(ctrlAttitude.YawCtrl), 7.0, 1.0, 10, 1000, lag_fc, lag_alpha, lead_fc,
                                     lead_alpha);

            init_angular_control_2nd(573,35,1,7,15,&rollCtrl);
            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);
            std::ostringstream oss;
            oss << std::put_time(&tm, "/var/log/rapidfdm/so3_log_%Y_%m_%d_%H_%M_%S.mat");
            auto file = oss.str();
            printf("Creating file %s...\n\n", file.c_str());
            pmat = matOpen(file.c_str(), "w");
        }

        void convert_euler_to_array(double quat[], double roll, double pitch, double yaw) {
            Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());
            Eigen::Quaterniond quat_sp = yawAngle * pitchAngle * rollAngle;
            quat[0] = quat_sp.w();
            quat[1] = quat_sp.x();
            quat[2] = quat_sp.y();
            quat[3] = quat_sp.z();
        }

        void so3_adaptive_controller::control_multirotor(float deltatime) {
            Eigen::Vector3d eul = quat2eulers(quat);
            QuatControlSetpoint quatControlSetpoint;
            static double yaw_angle_sp = 0;
            if (fabs(yaw_sp) > 0.05) {
                yaw_angle_sp = eul.z();
                quatControlSetpoint.yaw_rate = yaw_sp * M_PI;
                quatControlSetpoint.yaw_sp_is_rate = 1;
            } else {
                quatControlSetpoint.yaw_sp_is_rate = 0;
            }

            convert_euler_to_array(quatControlSetpoint.quat, roll_sp, pitch_sp, yaw_angle_sp);

            L1ControlAttitude(&ctrlAttitude, deltatime, &quatControlSetpoint, &sys);

            Eigen::AngleAxisd rot_u(0 * M_PI / 180, Eigen::Vector3d::UnitZ());
            Eigen::Vector3d u = Eigen::Vector3d(0, 0, 0);
            Eigen::Vector3d u_last = Eigen::Vector3d(0, 0, 0);
            angular_velocity_control_2nd(&rollCtrl,deltatime,&sys,roll_sp);
            u.x() = ctrlAttitude.u[0];
//            u.x() = rollCtrl.u;
            u.y() = ctrlAttitude.u[1];
            u.z() = ctrlAttitude.u[2];

            pwm[0] = (float) float_constrain((-u.x() + u.y() + u.z()) + throttle_sp, 0, 1);
            pwm[1] = (float) float_constrain((u.x() + u.y() - u.z()) + throttle_sp, 0, 1);
            pwm[2] = (float) float_constrain((u.x() - u.y() + u.z()) + throttle_sp, 0, 1);
            pwm[3] = (float) float_constrain((-u.x() - u.y() - u.z()) + throttle_sp, 0, 1);


        }

        void so3_adaptive_controller::control_fixedwing(float deltatime) {
            Eigen::Vector3d eul = quat2eulers(quat);
            QuatControlSetpoint quatControlSetpoint;
            static double yaw_angle_sp = 0;
            yaw_angle_sp = eul.z();
            quatControlSetpoint.yaw_rate = yaw_sp /10 * M_PI;
            quatControlSetpoint.yaw_sp_is_rate = 1;

            convert_euler_to_array(quatControlSetpoint.quat, roll_sp, pitch_sp, yaw_angle_sp);

            L1ControlAttitude(&ctrlAttitude, deltatime, &quatControlSetpoint, &sys);

//            angular_velocity_control_2nd(&rollCtrl,deltatime,&sys,roll_sp);
            Eigen::Vector3d u = Eigen::Vector3d(0, 0, 0);

            u.x() = ctrlAttitude.u[0];
//            u.x() = rollCtrl.u;
//            u.x() = roll_sp;
            u.y() = ctrlAttitude.u[1];
            u.z() = ctrlAttitude.u[2];

            pwm[0] = (float) float_constrain(u.x(),-1,1);
            pwm[1] = (float) float_constrain(-u.y(), -1, 1);
            pwm[2] = (float) float_constrain(throttle_sp, 0, 1);
            pwm[3] = (float) float_constrain(u.z(), -1, 1);

        }

        void so3_adaptive_controller::control_step(float deltatime) {
            static int count = 0;
            count++;
            deltatime = deltatime / 1000;
            static Eigen::Vector3d angular_rate_last = Eigen::Vector3d(0,0,0);
            Eigen::Vector3d angular_rate_dot = (angular_rate - angular_rate_last) / deltatime;
            angular_rate_last = angular_rate;
            copy_v3d_into_array(angular_rate, sys.angular_rate);
            copy_v3d_into_array(angular_rate_dot, sys.angular_rate_dot);
            sys.quat[0] = quat.w();
            sys.quat[1] = quat.x();
            sys.quat[2] = quat.y();
            sys.quat[3] = quat.z();

            control_fixedwing(deltatime);
//            control_multirotor(deltatime);

            aircraftNode->set_control_from_channels(pwm, 8);

//            ctrl_log.push_back(ctrlAttitude.PitchCtrl);
            ctrl_log.push_back(ctrlAttitude.RollCtrl);
            sys_log.push_back(sys);
            att_con_log.push_back(ctrlAttitude);

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
            //t 1 x 6 err 2 u 1 eta 1

            int cols = 22;
            pa1 = mxCreateDoubleMatrix(ctrl_log.size(), cols, mxREAL);
            for (int i = 0; i < ctrl_log.size(); i++) {
                const AdaptiveCtrlT &ctrlT = ctrl_log[i];
                set_value_mx_array(pa1, i, 0, ctrlT.t);
                for (int j = 0; j < 7; j++) {
                    set_value_mx_array(pa1, i, j + 1, ctrlT.x[j]);
                }
                set_value_mx_array(pa1, i, 8, ctrlT.err[0]);
                set_value_mx_array(pa1, i, 9, ctrlT.err[1]);
                set_value_mx_array(pa1, i, 10, ctrlT.u);
                set_value_mx_array(pa1, i, 11, ctrlT.eta);
                set_value_mx_array(pa1, i, 12, ctrlT.r);
                set_value_mx_array(pa1, i, 13, ctrlT.g[0]);
                for (int j = 0; j < 4; j++) {
                    set_value_mx_array(pa1, i, j + 14, att_con_log[i].quat[j]);
                }
                for (int j = 0; j < 4; j++) {
                    set_value_mx_array(pa1, i, j + 18, att_con_log[i].quat_sp[j]);
                }
            }
            char matname[100] = {0};
            sprintf(matname, "AdaptiveCtrlT_%d", log_number);
            matPutVariable(pmat, matname, pa1);
            mxDestroyArray(pa1);
            ctrl_log.clear();
            sys_log.clear();
            att_con_log.clear();
            log_number++;
        }
    }
}
