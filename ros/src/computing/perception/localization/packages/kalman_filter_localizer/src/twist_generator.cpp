#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <nav_msgs/Odometry.h>

class TwistGenerator
{
  public:
    TwistGenerator() : nh_(""), pnh_("~"), wheelbase_(2.79)
    {
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/can_twist", 1);
        pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status_converted", 1);
        sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &TwistGenerator::callbackVehicleStatus, this);
        
    };
    ~TwistGenerator(){};

  private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_vehicle_status_;
    ros::Publisher pub_twist_, pub_vehicle_status_;
    const double wheelbase_;

    void callbackVehicleStatus(const autoware_msgs::VehicleStatus &msg)
    {
        const double kmph2mps = 1000.0 / 3600.0;

        geometry_msgs::TwistStamped twist_stamped;
        twist_stamped.header = msg.header;
        twist_stamped.twist.linear.x = msg.speed * kmph2mps;
        twist_stamped.twist.angular.z = twist_stamped.twist.linear.x * std::tan(msg.angle) / wheelbase_;
        pub_twist_.publish(twist_stamped);

        autoware_msgs::VehicleStatus vs;
        vs.speed *= kmph2mps;
        pub_vehicle_status_.publish(vs);
    }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "twist_generator");
  TwistGenerator obj;
  ros::spin();
  return 0;
};