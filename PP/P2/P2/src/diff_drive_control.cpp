#include <turtlebot3_diff_drive_exercise/diff_drive_control.h>

#define _USE_MATH_DEFINES
#include <tf/tf.h>
using namespace std;


namespace turtlebot3
{
DiffDriveControl::DiffDriveControl(ros::NodeHandle& nh)
  : is_active_(false)
  , first_heading_alignment_done_(false),
    goal_position_reached_(false)
{
  max_linear_vel_ = nh.param("diff_drive/max_linear_vel", 1.0);
  max_linear_acc_ = nh.param("diff_drive/max_linear_acc", 1.0);
  max_angular_vel_ = nh.param("diff_drive/max_angular_vel", 1.0);
  max_angular_acc_ = nh.param("diff_drive/max_angular_acc", 1.0);

  pos_tolerance_ = nh.param("diff_drive/pos_tolerance", 0.01);
  angular_tolerance_ = nh.param("diff_drive/angular_tolerance", 0.01);

  allow_backward_ = nh.param("diff_drive/allow_backward", true);

  p_gain_ = nh.param("diff_drive/p_gain", 1.0);
  last_erro_theta=0;
  turn_first=false;
  distance_null=false;
  backward=false;
  first=true;
}

void DiffDriveControl::reset()
{
  is_active_ = false;
  first_heading_alignment_done_ = false;
  goal_position_reached_ = false;
  goal_pose_ = geometry_msgs::Pose();
  turn_first=false;
  distance_null=false;
  last_erro_theta=0;
  last_erro_theta2=0;
  backward=false;
  first=true;
}

void DiffDriveControl::setGoalPose(const geometry_msgs::Pose& goal)
{
  is_active_ = true;
  goal_pose_ = goal;
}

void DiffDriveControl::stop()
{
  reset();
}

bool DiffDriveControl::isActive() const
{
  return is_active_;
}

//*******//

//*******//
geometry_msgs::Twist DiffDriveControl::computeTwist(const ros::Duration& time_diff, const geometry_msgs::Pose& robot_pose, const nav_msgs::Odometry& odom)
{
  geometry_msgs::Twist twist;

  double robot_yaw = tf::getYaw(robot_pose.orientation);
  double goal_yaw = tf::getYaw(goal_pose_.orientation);

  /// | Implement differential drive control here |
  /// v                                           v

  double distance;
  double erro_theta2;
  double diff_x=goal_pose_.position.x-robot_pose.position.x;
  double diff_y=goal_pose_.position.y-robot_pose.position.y;
  twist.linear.y=0;
  twist.linear.x=0;
  twist.linear.z=0;
  twist.angular.x=0;
  twist.angular.y=0;
  twist.angular.z=0;

  if(goal_position_reached_==false)
  {
      double erro_theta=atan2(diff_y,diff_x)-robot_yaw;


      erro_theta=theta_corr(erro_theta);
      distance=sqrt(pow(diff_x,2)+pow(diff_y,2));
      if(distance>pos_tolerance_/5)
      {
          distance_null=false;
      }
      else
      {
          distance_null=true;
      }
      double dt = time_diff.toSec();

      /***********backward************/
      if((abs(erro_theta)>3*M_PI/4))
      {
         if(allow_backward_)
         {
             backward=true;
             if(erro_theta>0)
             {
               erro_theta = erro_theta-M_PI;
             }
             else
             {
               erro_theta = M_PI+erro_theta;
             }
         }
      }
      else {
          backward=false;
      }
      /*************************/
      twist.angular.z=compute_omega(erro_theta,odom.twist.twist.angular.z,dt);
      last_erro_theta=erro_theta;
      if((turn_first==true)&&(distance_null==false))  //
      {
          if(distance>pos_tolerance_/5)
          {
              double v= (M_PI-erro_theta)/M_PI*max_linear_vel_;
              if(backward)
              {
                  v=-v;
              }
              if(distance<(1.25*pos_tolerance_))
              {
                  v=v/2;
              }
              twist.linear.x=compute_v(odom.twist.twist.linear.x,dt,v);
          }
          else
          {
              twist.linear.x=0;
              distance_null=true;
          }

      }

      if(distance_null)
      {
          erro_theta2=goal_yaw-robot_yaw;
//          if(first)
//          {
//              cout<<"erro_theta2: "<<erro_theta2<<endl;
//          }
          erro_theta2=theta_corr(erro_theta2);
//          if(first)
//          {
//              cout<<"erro_theta3: "<<erro_theta2<<endl;
//              cout<<"angular.z: "<<odom.twist.twist.angular.z<<endl;
//          };
          twist.angular.z=compute_omega(erro_theta2,odom.twist.twist.angular.z,dt);
      }
      if((abs(goal_yaw-robot_yaw)<angular_tolerance_)&&(distance<pos_tolerance_))
      {
          goal_position_reached_=true;
          cout<<"get"<<endl;
          reset();
      }
  }


  
  /// ^                                           ^
  /// | ------ Differential drive control end --- |

  if (is_active_)
  {
    return twist;
  }
  else
    return geometry_msgs::Twist(); // stops robot
}

/// | ------ Additional Methods ------ |
/// v                                  v
double DiffDriveControl::compute_v(double V, double time_diff ,double Vdes)
{
    double vel = V;
    double acc_=0;
    if(abs(V-Vdes)<0.001)
    {
        return V;
    }
    else
    {
        acc_=(Vdes-V)/time_diff;
    }
    if(acc_>max_linear_acc_)
    {
        acc_=max_linear_acc_;
    }
    else if(acc_<-max_linear_acc_)
    {
        acc_=-max_linear_acc_;
    }
    vel +=acc_*time_diff;
    if(vel>max_linear_vel_)
    {
        vel=max_linear_vel_;
    }
    else if (vel<-max_linear_vel_)
    {
        vel=-max_linear_vel_;
    }
    return vel;
}
double DiffDriveControl::compute_omega(double theta ,double om, double time_diff)
{
    double w=om;
    double erro=theta;
    double wdes=0;
    double acc;
    if(abs(theta)<angular_tolerance_)
    {
        turn_first=true;
        return w=0;
    }
    if((abs(theta)>M_PI/4)&&(distance_null==false))
    {
        turn_first=false;
    }
    if((turn_first!=true)||(distance_null==true))
    {
        erro=p_gain_*erro;
        if (abs(erro)>0.35)
        {
          if(erro>0)
          {
            wdes=max_angular_vel_;
          }
          else {
            wdes=-max_angular_vel_;
          }
        }
        else {
            if(erro>0)
            {
              wdes=max_angular_vel_/5;
            }
            else {
              wdes=-max_angular_vel_/5;
            }
        }

    }
    else if(distance_null==false)
    {
        erro=p_gain_*erro+0.1*(last_erro_theta-erro);
        if (abs(erro)<0.2)
        {
          if(erro>0)
          {
            erro=0.2;
          }
          else {
            erro=-0.2;
          }
        }
        wdes=erro/M_PI*max_angular_vel_;
    }

    acc=(wdes-w)/time_diff;

    if(acc>max_angular_acc_)
    {
        acc=max_angular_acc_;
    }
    else if(acc<(-max_angular_acc_))
    {
        acc=(-max_angular_acc_);
    }
    w +=acc*time_diff;
    if(w>max_angular_vel_)
    {
        w=max_angular_vel_;
    }
    else if (w<-max_angular_vel_)
    {
        w=-max_angular_vel_;
    }

    return w;

}

double DiffDriveControl:: theta_corr(double theta)
{
    double th=theta;
    if(abs(th)>M_PI)
    {
        if(th>0)
        {
            th -=2*M_PI;
        }
        else {
            th +=2*M_PI;
        }
    }
    return th;
}
/// ^                                  ^
/// | ------ Additional Methods end -- |

} // namespace
