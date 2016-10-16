#include "WOdometer.h"

namespace ORB_SLAM2 {
    void WOdometer::generateOdom(const std::vector<KeyFrame*> &vKeyFrames,
                                                              std::vector<Eigen::Vector3d> &vConstraints,
                                                              std::vector<Eigen::Matrix3d> &vCovariance)
    {
        for(int i = vKeyFrames.size()-1; i > 0; i--)
        {
            KeyFrame* lastKF = vKeyFrames[i];
            KeyFrame* curKF = vKeyFrames[i-1];
            double t0 = lastKF->_timestep;
            double t1 = curKF->_timestep;
            WheelVector vWheel;
            getWheelDatas(t0, t1, vWheel);
            Eigen::Vector3d relativePose = Eigen::Vector3d::Zero();
            Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
            for(auto it:vWheel)
            {
                double deltat = it.first - t0;
                t0 = it.first;
                WheelVelocitys velocitys = it.second;
                double vl = velocitys.first*0.035;
                double vr = velocitys.second*0.035;
                double v = (vl+vr)/2;
                double w = (vr-vl)/0.23;
                double sigma1 = vl*0.02;
                double sigma2 = vr*0.02;
                double s12 = sigma1*sigma1;
                double s22 = sigma2*sigma2;

                relativePose[0] = relativePose[0]+v*deltat*cos(relativePose[2]);
                relativePose[1] = relativePose[1]+v*deltat*sin(relativePose[2]);
                relativePose[2] = relativePose[2]+w*deltat;

                Eigen::Matrix3d Phi = Eigen::Matrix3d::Identity();
                Phi(0,2) = -v*deltat*sin(relativePose[2]);
                Phi(1,2) = v*deltat*cos(relativePose[2]);
                Eigen::Matrix<double, 3, 2> G = Eigen::Matrix<double, 3 ,2>::Zero();
                G(0,0) = -deltat*cos(relativePose[2]);
                G(1,0) = -deltat*sin(relativePose[2]);
                G(2,1) = -deltat;
                Eigen::Matrix2d Q = Eigen::Matrix2d::Zero();
                Q(0,0) = (s12 + s22)/4; Q(0,1) = (s12-s22)/(2*0.23);
                Q(1,0) = (s12-s22)/(2*0.23); Q(1,1) = (s12 + s22)/(0.23*0.23);

                covariance = Phi*covariance*Phi.transpose() + G*Q*G.transpose();
            }

            vConstraints.push_back(relativePose);
            vCovariance.push_back(covariance);
        }
    }

    void WOdometer::getWheelDatas(double &lastT, double &curT, WheelVector &wheel)
    {
        std::ifstream in("/home/doom/thesis_result/wheel.txt");
        for(int i = 0; i < 33032; i++){
            double temp;
            in >> temp;
            if(temp > curT)
                break;
            if((temp >= lastT) && (temp <= curT))
            {
                WheelElements ptemp;
                ptemp.first = temp;
                WheelVelocitys temp1;
                in >> temp;
                temp1.first = temp;
                in >> temp;
                temp1.second = temp;
                ptemp.second = temp1;
                wheel.push_back(ptemp);
            }
            else
            {
                in >> temp; in >> temp;
            }
        }
    }
}
