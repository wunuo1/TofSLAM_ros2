#ifndef MAPPING_ODOM_H
#define MAPPING_ODOM_H

namespace zjloc
{

    struct Odom
    {
        Odom() {}
        Odom(double timestamp, double left_pulse, double right_pulse)
            : timestamp_(timestamp), left_pulse_(left_pulse), right_pulse_(right_pulse) {}

        double timestamp_ = 0.0;
        double left_pulse_ = 0.0; // 左右轮的单位时间转过的脉冲数
        double right_pulse_ = 0.0;
    };

    struct Odompos
    {
        Odompos() {}
        Odompos(double timestamp, const Vec3d &pos, const Vec4d &rot, double vx, double vy)
            : timestamp_(timestamp), pos_(pos), rot_(rot), vx_(vx), vy_(vy) {}

        double timestamp_ = 0.0;
        Vec3d pos_ = Vec3d::Zero();
        Vec4d rot_ = Vec4d::Zero(); //wxyz
        double vx_ = 0.0;
        double vy_ = 0.0;
        double vz_ = 0.0;
    };

} // namespace zjloc
using OdomposPtr = std::shared_ptr<zjloc::Odompos>;
#endif // MAPPING_ODOM_H
