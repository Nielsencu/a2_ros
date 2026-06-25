#include <chrono>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
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
// The same handoff also fires on a timeout: if num_mins_before_return > 0 and
// exploration has been running that long without TARE reporting finished, the
// node switches to FAR anyway so the robot can't get stuck exploring forever.
// The clock starts when the local planner publishes its first real (multi-pose)
// /path -- i.e. when the robot actually starts moving -- not at node startup, so
// the planner's path-primitive loading time is not counted against the budget.
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

    // Safety timeout: if exploration runs this many minutes without TARE
    // reporting it has finished, force the return-home handoff anyway so we
    // never explore forever. <= 0 disables it (return only on /exploration_finish).
    // The clock starts when the robot first moves (see pathCallback), not now.
    const double num_mins_before_return =
      declare_parameter<double>("num_mins_before_return", 0.0);
    return_timeout_s_ = num_mins_before_return * 60.0;
    // explore_start_time_ is set later, when the robot actually starts moving
    // (first multi-pose /path), so loading/standby time isn't counted.

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

    // Local-planner path. The first multi-pose path means the planner has loaded
    // its primitives, has a waypoint, and committed to a trajectory -> the robot
    // is starting to move, so this is when the exploration clock starts.
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
      "path", 5,
      std::bind(&HomeReturnMux::pathCallback, this, std::placeholders::_1));

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
      [this]() {
        if (returning_home_) {
          if (!reached_home_) publishGoal();
        } else if (mission_started_ && return_timeout_s_ > 0.0 &&
                   (now() - explore_start_time_).seconds() >= return_timeout_s_) {
          triggerReturnHome("Exploration time limit reached");
        }
      });

    // Periodic status line: elapsed mission time + current state. <= 0 disables.
    const double status_period = declare_parameter<double>("status_print_period_s", 5.0);
    if (status_period > 0.0) {
      status_timer_ = create_wall_timer(
        std::chrono::duration<double>(status_period),
        [this]() { printStatus(); });
    }
  }

private:
  void finishCallback(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data) triggerReturnHome("Exploration finished");
  }

  // Start the exploration clock on the first real (multi-pose) local path. A
  // single-pose path is the planner's "stop" placeholder (no feasible path or
  // goal reached) and does not mean the robot is moving, so it is ignored.
  void pathCallback(const nav_msgs::msg::Path::SharedPtr msg)
  {
    if (mission_started_ || msg->poses.size() <= 1) return;
    mission_started_    = true;
    explore_start_time_ = now();
    RCLCPP_INFO(get_logger(),
                "Robot started moving (first local path); exploration timer started.");
  }

  // Latch into return-home mode and send the home goal. Idempotent: whichever
  // trigger fires first (TARE finish or the time limit) wins; later calls are
  // no-ops, keeping the handoff one-way.
  void triggerReturnHome(const std::string & reason)
  {
    if (returning_home_) return;
    returning_home_ = true;
    RCLCPP_INFO(get_logger(),
                "%s -> switching to FAR planner, returning home to "
                "(%.2f, %.2f, %.2f) in frame '%s'.",
                reason.c_str(), home_x_, home_y_, home_z_, world_frame_.c_str());
    publishGoal();  // send the goal immediately, then keep re-sending on timer
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

  void printStatus()
  {
    if (!mission_started_) {
      RCLCPP_INFO(get_logger(),
                  "state=WAITING_TO_MOVE (timer starts on first local path)");
      return;
    }
    const double elapsed = (now() - explore_start_time_).seconds();
    if (returning_home_) {
      RCLCPP_INFO(get_logger(), "[%6.1fs] state=RETURNING_HOME reached_home=%s",
                  elapsed, reached_home_ ? "true" : "false");
    } else if (return_timeout_s_ > 0.0) {
      double remaining = return_timeout_s_ - elapsed;
      if (remaining < 0.0) remaining = 0.0;
      RCLCPP_INFO(get_logger(), "[%6.1fs] state=EXPLORING (forced return in %.1fs)",
                  elapsed, remaining);
    } else {
      RCLCPP_INFO(get_logger(), "[%6.1fs] state=EXPLORING", elapsed);
    }
  }

  double home_x_{0.0};
  double home_y_{0.0};
  double home_z_{0.0};
  std::string world_frame_;
  double return_timeout_s_{0.0};
  rclcpp::Time explore_start_time_;
  bool mission_started_{false};
  bool returning_home_{false};
  bool reached_home_{false};

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr way_point_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tare_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr far_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr finish_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reach_sub_;
  rclcpp::TimerBase::SharedPtr goal_timer_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HomeReturnMux>());
  rclcpp::shutdown();
  return 0;
}
