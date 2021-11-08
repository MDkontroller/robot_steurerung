#include    <ros/ros.h>
#include    <std_msgs/Float64.h>
#include    <geometry_msgs/Twist.h>

namespace DiffController
{

typedef enum
{
    LEFT = 0,
    RIGHT = 1
} Diff_Motor;

class TwistToDiffDrive
{
    public:
        TwistToDiffDrive(float wheel_distance, float wheel_diameter, float transmission_factor, float motor_direction_left, float motor_direction_right);
        
        float calcMotorSpeed(Diff_Motor side, float lin_x, float ang_z);

    private:
        float _wheel_distance, _wheel_diameter, _trans_factor;
        float _motor_direction[2];
        float _sideFactor[2] = { -1.0, 1.0 };
};

} // namespace DiffController