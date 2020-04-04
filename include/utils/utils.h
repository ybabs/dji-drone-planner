#include <cmath>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3Stamped.h>

#define PI (double) 3.141592653589793
#define C_EARTH (double)6378137.0


static double DegToRad(double degree)
{
    return degree * (PI/180.0);
}

static double RadToDeg(double rad)
{
    return rad *  (180.0/PI);
}

static geometry_msgs::Vector3 toEulerAngle(geometry_msgs::Quaternion quat)
{
    geometry_msgs::Vector3 ans;

    tf::Matrix3x3 R_FLU2ENU(tf::Quaternion(quat.x, quat.y, quat.z, quat.w));
    R_FLU2ENU.getRPY(ans.x, ans.y, ans.z);
    return ans;

}
