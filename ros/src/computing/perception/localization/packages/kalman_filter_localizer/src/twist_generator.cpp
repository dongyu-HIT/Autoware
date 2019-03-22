
#include <ros/ros.h>

#include <geometry_msgs/TwistStamped.h>
#include <autoware_msgs/VehicleStatus.h>
#include <nav_msgs/Odometry.h>

class TwistGenerator
{
  public:
    TwistGenerator() : nh_(""), pnh_("~"), wheelbase_(2.9)
    {
        pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("can_twist", 1);
        pub_vehicle_status_ = nh_.advertise<autoware_msgs::VehicleStatus>("/vehicle_status", 1);
        // sub_vehicle_status_ = nh_.subscribe("/vehicle_status", 1, &TwistGenerator::callbackVehicleStatus, this);
        sub_odom_ = nh_.subscribe("/vehicle/odom", 1, &TwistGenerator::callbackOdom, this);
    };
    ~TwistGenerator(){};

  private:
    ros::NodeHandle nh_, pnh_;
    ros::Subscriber sub_vehicle_status_, sub_odom_;
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
    }
    void callbackOdom(const nav_msgs::Odometry &msg)
    {
      geometry_msgs::TwistStamped twist;
      twist.header.stamp = msg.header.stamp;
      twist.header.frame_id = "base_link";
      twist.twist = msg.twist.twist;
      pub_twist_.publish(twist);

      double kappa;
      if (std::fabs(msg.twist.twist.linear.x) > 0.1)
      {
        kappa = msg.twist.twist.angular.z / msg.twist.twist.linear.x;
      }
      else
      {
        kappa = 0.0;
      }
      autoware_msgs::VehicleStatus vs;
      vs.header.stamp = msg.header.stamp;
      vs.header.frame_id = "base_link";
      vs.speed = msg.twist.twist.linear.x;
      vs.angle = std::atan(kappa * wheelbase_);
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
