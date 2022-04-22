/*
 * X-Y-Z IMU data needed!
 */

#include "sensor_msgs/Imu.h"
#include "Eigen/Eigen"

class imuIntegral{
private:
    Eigen::Vector3d gravity{0, 0, -9.8};

    Eigen::Vector3d P_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d V_last = Eigen::Vector3d::Zero();
    Eigen::Quaterniond Q_last = Eigen::Quaterniond::Identity();
public:
    Eigen::Vector3d P = Eigen::Vector3d::Zero();
    Eigen::Vector3d V = Eigen::Vector3d::Zero();
    Eigen::Quaterniond Q = Eigen::Quaterniond::Identity();

    void integral(sensor_msgs::Imu msg, double dt)
    {
        double ax = msg.linear_acceleration.x;
        double ay = msg.linear_acceleration.y;
        double az = msg.linear_acceleration.z;
        Eigen::Vector3d linear{ax, ay, az};
        double gx = msg.angular_velocity.x;
        double gy = msg.angular_velocity.y;
        double gz = msg.angular_velocity.z;
        Eigen::Vector3d angle{gx, gy, gz};

        V = V_last + (gravity * dt) + (Q_last.matrix() * linear * dt);
        P = P_last + (V_last * dt) + (0.5 * gravity * dt * dt ) + (0.5 * dt * dt * Q_last.matrix() * linear);
        Q = Q_last * Eigen::Quaterniond(1, 0.5 * angle(0) * dt, 0.5 * angle(1) * dt, 0.5 * angle(2) * dt);

        P_last = P;
        V_last = V;
        Q_last = Q;
    }

};

