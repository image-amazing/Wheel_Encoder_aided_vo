#ifndef WODOMETER_H
#define WODOMETER_H

#include "KeyFrame.h"

#include <vector>
#include <fstream>
#include <Eigen/Core>

typedef std::vector<std::pair<double,std::pair<double, double> > > WheelVector;
typedef std::pair<double,std::pair<double, double> > WheelElements;
typedef std::pair<double, double> WheelVelocitys;

namespace ORB_SLAM2
{
class WOdometer
{
    public:
        void static generateOdom(const std::vector<KeyFrame*> & vKeyFrames,
                                                     std::vector<Eigen::Vector3d> & vConstraints,
                                                     std::vector<Eigen::Matrix3d> & vCovariance);
        void static getWheelDatas(double & lastT, double & curT, WheelVector & wheel);
};
}
#endif
