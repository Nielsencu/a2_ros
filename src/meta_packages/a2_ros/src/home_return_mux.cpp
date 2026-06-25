#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

// Waypoint mux for the TARE -> FAR return-home handoff.
//
// Both TARE and FAR publish PointStamped waypoints for the local planner, but
// they cannot share /way_point. In exploration.launch.py each planner's output
// is remapped to a private topic (/way_point_tare, /way_point_far) and this node
// forwards exactly one of them to the real /way_point.
//
// While exploring, TARE drives. When TARE reports /exploration_finish == true
// (it has finished coverage and is heading back), this node latches into
// "return home" mode: it forwards FAR's waypoints instead, and publishes the
// home goal (default 0,0,0 in the map frame) to /goal_point so FAR plans a path
// back. FAR ignores goals until its visibility graph is initialised, so the goal
// is re-published on a timer until FAR reports /far_reach_goal_status == true.
//
// The handoff is one-way: once returning home, the node stays in FAR mode.
class HomeReturnMux : public rclcpp::Node {
public:
  HomeReturnMux() : Node("home_return_mux")
  {
    home_x_      = declare_parameter<double>("home_x", 0.0);
    home_y_      = declare_parameter<double>("home_y", 0.0);
    home_z_      = declare_parameter<double>("home_z", 0.0);
    world_frame_ = declare_parameter<std::string>("world_frame", "map");
    const double repub_period = declare_parameter<double>("goal_repub_period_s", 1.0);

    way_point_pub_ = create_publisher<geometry_msgs::msg::PointStamped>("way_point", 5);
    goal_pub_      = create_publisher<geometry_msgs::msg::PointStamped>("goal_point", 5);

    tare_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "way_point_tare", 5,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!returning_home_) way_point_pub_->publish(*msg);
      });

    far_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "way_point_far", 5,
      [this](const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (returning_home_) way_point_pub_->publish(*msg);
      });

    finish_sub_ = create_subscription<std_msgs::msg::Bool>(
      "exploration_finish", 5,
      std::bind(&HomeReturnMux::finishCallback, this, std::placeholders::_1));

    reach_sub_ = create_subscription<std_msgs::msg::Bool>(
      "far_reach_goal_status", 5,
      [this](const std_msgs::msg::Bool::SharedPtr msg) {
        if (returning_home_ && msg->data && !reached_home_) {
          reached_home_ = true;
          RCLCPP_INFO(get_logger(),
                      "FAR reports home reached; stopping goal re-publish.");
        }
      });

    goal_timer_ = create_wall_timer(
      std::chrono::duration<double>(repub_period),
      [this]() { if (returning_home_ && !reached_home_) publishGoal(); });
  }

private:
  void finishCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data && !returning_home_) {
      returning_home_ = true;
      RCLCPP_INFO(get_logger(),
                  "Exploration finished -> switching to FAR planner, returning "
                  "home to (%.2f, %.2f, %.2f) in frame '%s'.",
                  home_x_, home_y_, home_z_, world_frame_.c_str());
      publishGoal();  // send the goal immediately, then keep re-sending on timer
    }
  }

  void publishGoal()
  {
    geometry_msgs::msg::PointStamped goal;
    goal.header.stamp = now();
    goal.header.frame_id = world_frame_;
    goal.point.x = home_x_;
    goal.point.y = home_y_;
    goal.point.z = home_z_;
    goal_pub_->publish(goal);
  }

  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  std::string world_frame_;
  bool returning_home_{false};
  bool reached_home_{false};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tare_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr far_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reach_sub_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomeReturnMux>());
  rclcpp::shutdown();
  return 0;
}
