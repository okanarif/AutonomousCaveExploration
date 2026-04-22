// =============================================================================
// Sampling-Based Trajectory Planner
// =============================================================================
//
// Generates collision-free trajectories using 5th-order (quintic) polynomials.
// Rather than the conventional "plan a path then smooth it" pipeline, this node
// works directly in trajectory space:
//
//   1. Sample N candidate quintic trajectories with varied durations and
//      lateral/vertical offsets around the goal.
//   2. Collision-check each trajectory densely against the OctoMap.
//   3. Score feasible trajectories by time, clearance, and goal deviation.
//   4. Execute the best one.
//
// If no single-segment trajectory is collision-free, the node recursively
// splits via intermediate waypoints to build multi-segment trajectories.
//
// Operates in two modes:
//   - NAVIGATE_TO_CAVE: receives waypoints via ExecuteTrajectory service
//   - EXPLORE_CAVE:     periodically replans toward the frontier goal
//
// All parameters (sampling budget, speed profiles, topic/service names)
// are loaded from a YAML config file at startup.
// =============================================================================

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <random>
#include <cmath>
#include <algorithm>
#include <limits>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/empty.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <Eigen/Dense>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "mission_interfaces/srv/execute_trajectory.hpp"

using namespace std::chrono_literals;

// =============================================================================
// QuinticPoly — 5th-order polynomial: p(t) = c0 + c1*t + … + c5*t^5
// Boundary conditions: position, velocity, acceleration at t=0 and t=T.
// =============================================================================
struct QuinticPoly {
    double c[6] = {};

    double pos(double t) const {
        return c[0] + c[1]*t + c[2]*t*t + c[3]*t*t*t + c[4]*t*t*t*t + c[5]*t*t*t*t*t;
    }
    double vel(double t) const {
        return c[1] + 2*c[2]*t + 3*c[3]*t*t + 4*c[4]*t*t*t + 5*c[5]*t*t*t*t;
    }
    double acc(double t) const {
        return 2*c[2] + 6*c[3]*t + 12*c[4]*t*t + 20*c[5]*t*t*t;
    }

    // Solve for c3, c4, c5 given boundary conditions
    static QuinticPoly solve(double p0, double v0, double a0,
                             double pT, double vT, double aT, double T) {
        QuinticPoly q;
        q.c[0] = p0;
        q.c[1] = v0;
        q.c[2] = a0 / 2.0;

        double T2 = T * T, T3 = T2 * T, T4 = T3 * T, T5 = T4 * T;

        Eigen::Matrix3d A;
        A <<   T3,    T4,    T5,
             3*T2,  4*T3,  5*T4,
             6*T,  12*T2, 20*T3;

        Eigen::Vector3d b;
        b(0) = pT - p0 - v0*T - (a0/2.0)*T2;
        b(1) = vT - v0 - a0*T;
        b(2) = aT - a0;

        Eigen::Vector3d x = A.colPivHouseholderQr().solve(b);
        q.c[3] = x(0);
        q.c[4] = x(1);
        q.c[5] = x(2);
        return q;
    }
};

// One segment: independent quintic polynomials for x, y, z over duration T
struct TrajSegment {
    QuinticPoly px, py, pz;
    double T;
};

// Full trajectory = ordered list of segments
struct Trajectory {
    std::vector<TrajSegment> segments;
    double total_time = 0.0;
    double score      = 0.0;  // Lower is better

    void computeTotalTime() {
        total_time = 0.0;
        for (auto& s : segments) total_time += s.T;
    }
};

// =============================================================================
// SamplingPlannerNode
// =============================================================================
class SamplingPlannerNode : public rclcpp::Node {
public:
    SamplingPlannerNode()
    : Node("sampling_planner_node"),
      rng_(std::random_device{}())
    {
        // ------------------------------------------------------------------
        // Declare parameters with defaults (overridden by YAML config)
        // ------------------------------------------------------------------
        // Sampling
        this->declare_parameter("num_samples",            10);
        this->declare_parameter("min_duration_factor",    1.0);
        this->declare_parameter("max_duration_factor",    1.8);
        this->declare_parameter("lateral_spread",         3.0);
        this->declare_parameter("max_recursion_depth",    4);

        // Safety & collision
        this->declare_parameter("safety_radius",          2.0);
        this->declare_parameter("collision_check_dt",     0.2);

        // Speed profiles
        this->declare_parameter("navigate_to_cave_speed", 10.0);
        this->declare_parameter("cave_exploration_speed",  4.0);

        // Planning behaviour
        this->declare_parameter("planning_frequency",     2.0);
        this->declare_parameter("lookahead_distance",    15.0);
        this->declare_parameter("terminal_speed_fraction", 0.7);
        this->declare_parameter("goal_debounce_distance",  5.0);
        this->declare_parameter("min_goal_distance",       5.0);

        // Topic names
        this->declare_parameter<std::string>("octomap_topic",             "octomap_binary");
        this->declare_parameter<std::string>("pose_topic",                "pose_est");
        this->declare_parameter<std::string>("frontier_goal_topic",       "frontier_goal");
        this->declare_parameter<std::string>("state_mode_topic",          "stm_mode");
        this->declare_parameter<std::string>("odom_topic",                "current_state_est");
        this->declare_parameter<std::string>("desired_state_topic",       "desired_state");
        this->declare_parameter<std::string>("trajectory_complete_topic", "trajectory_complete");
        this->declare_parameter<std::string>("planned_trajectory_topic",  "planned_trajectory");

        // Service names
        this->declare_parameter<std::string>("start_navigation_service",  "start_navigation");

        // ------------------------------------------------------------------
        // Load all parameters
        // ------------------------------------------------------------------
        num_samples_            = this->get_parameter("num_samples").as_int();
        min_dur_factor_         = this->get_parameter("min_duration_factor").as_double();
        max_dur_factor_         = this->get_parameter("max_duration_factor").as_double();
        lateral_spread_         = this->get_parameter("lateral_spread").as_double();
        max_recursion_          = this->get_parameter("max_recursion_depth").as_int();
        safety_radius_          = this->get_parameter("safety_radius").as_double();
        collision_check_dt_     = this->get_parameter("collision_check_dt").as_double();
        navigate_to_cave_speed_ = this->get_parameter("navigate_to_cave_speed").as_double();
        cave_exploration_speed_ = this->get_parameter("cave_exploration_speed").as_double();
        planning_freq_          = this->get_parameter("planning_frequency").as_double();
        lookahead_distance_     = this->get_parameter("lookahead_distance").as_double();
        terminal_speed_frac_    = this->get_parameter("terminal_speed_fraction").as_double();
        goal_debounce_dist_     = this->get_parameter("goal_debounce_distance").as_double();
        min_goal_distance_      = this->get_parameter("min_goal_distance").as_double();

        std::string octomap_topic       = this->get_parameter("octomap_topic").as_string();
        std::string pose_topic          = this->get_parameter("pose_topic").as_string();
        std::string frontier_goal_topic = this->get_parameter("frontier_goal_topic").as_string();
        std::string state_mode_topic    = this->get_parameter("state_mode_topic").as_string();
        std::string odom_topic          = this->get_parameter("odom_topic").as_string();
        std::string desired_state_topic = this->get_parameter("desired_state_topic").as_string();
        std::string traj_complete_topic = this->get_parameter("trajectory_complete_topic").as_string();
        std::string planned_traj_topic  = this->get_parameter("planned_trajectory_topic").as_string();
        std::string start_nav_service   = this->get_parameter("start_navigation_service").as_string();

        // ------------------------------------------------------------------
        // Subscribers
        // ------------------------------------------------------------------
        auto octomap_qos = rclcpp::QoS(1).transient_local().reliable();
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic, octomap_qos,
            std::bind(&SamplingPlannerNode::octomapCallback, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 1,
            std::bind(&SamplingPlannerNode::poseCallback, this, std::placeholders::_1));

        goal_sub_ = this->create_subscription<geometry_msgs::msg::Point>(
            frontier_goal_topic, 1,
            std::bind(&SamplingPlannerNode::goalCallback, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            state_mode_topic, 1,
            std::bind(&SamplingPlannerNode::stateCallback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&SamplingPlannerNode::odomCallback, this, std::placeholders::_1));

        // ------------------------------------------------------------------
        // Publishers
        // ------------------------------------------------------------------
        desired_pub_   = this->create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
            desired_state_topic, 10);
        complete_pub_  = this->create_publisher<std_msgs::msg::Empty>(traj_complete_topic, 10);
        traj_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(planned_traj_topic, 10);

        // ------------------------------------------------------------------
        // Service server (used during NAVIGATE_TO_CAVE phase)
        // ------------------------------------------------------------------
        start_nav_srv_ = this->create_service<mission_interfaces::srv::ExecuteTrajectory>(
            start_nav_service,
            std::bind(&SamplingPlannerNode::startNavigationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ------------------------------------------------------------------
        // Timers
        // ------------------------------------------------------------------
        traj_timer_ = this->create_wall_timer(
            50ms, std::bind(&SamplingPlannerNode::trajTimerCallback, this));  // 20 Hz execution

        plan_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / planning_freq_),
            std::bind(&SamplingPlannerNode::planTimerCallback, this));

        RCLCPP_INFO(this->get_logger(),
            "Sampling planner initialized (samples=%d, safety_r=%.1f, nav_speed=%.1f, expl_speed=%.1f)",
            num_samples_, safety_radius_, navigate_to_cave_speed_, cave_exploration_speed_);
    }

    ~SamplingPlannerNode() {
        if (octree_) delete octree_;
    }

private:
    // ---- Tuneable parameters ------------------------------------------------
    int    num_samples_;
    double min_dur_factor_;
    double max_dur_factor_;
    double lateral_spread_;
    int    max_recursion_;
    double safety_radius_;
    double navigate_to_cave_speed_;
    double cave_exploration_speed_;
    double collision_check_dt_;
    double planning_freq_;
    double lookahead_distance_;
    double terminal_speed_frac_;
    double goal_debounce_dist_;
    double min_goal_distance_;

    // ---- ROS interfaces -----------------------------------------------------
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr     octomap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr       goal_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr           state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr desired_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr               complete_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                traj_path_pub_;
    rclcpp::Service<mission_interfaces::srv::ExecuteTrajectory>::SharedPtr start_nav_srv_;
    rclcpp::TimerBase::SharedPtr traj_timer_;
    rclcpp::TimerBase::SharedPtr plan_timer_;

    // ---- Runtime state ------------------------------------------------------
    octomap::OcTree* octree_ = nullptr;
    geometry_msgs::msg::PoseStamped current_pose_;
    geometry_msgs::msg::Point       current_goal_;
    bool pose_received_   = false;
    bool goal_received_   = false;
    bool octree_received_ = false;
    bool active_          = false;   // True when stm_mode == EXPLORE_CAVE
    bool goal_changed_    = false;
    bool odom_received_   = false;

    Eigen::Vector3d current_velocity_ = Eigen::Vector3d::Zero();
    double last_yaw_ = 0.0;

    // ---- Active trajectory --------------------------------------------------
    Trajectory active_traj_;
    bool   has_trajectory_ = false;
    bool   finished_pub_   = false;
    rclcpp::Time traj_start_time_;

    std::mt19937 rng_;

    // =========================================================================
    // Subscriber callbacks
    // =========================================================================

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if (octree_) delete octree_;
        auto* tree = octomap_msgs::msgToMap(*msg);
        octree_ = dynamic_cast<octomap::OcTree*>(tree);
        if (octree_) octree_received_ = true;
    }

    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_  = *msg;
        pose_received_ = true;
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_velocity_ << msg->twist.twist.linear.x,
                             msg->twist.twist.linear.y,
                             msg->twist.twist.linear.z;
        odom_received_ = true;
    }

    void goalCallback(const geometry_msgs::msg::Point::SharedPtr msg) {
        if (!active_) return;

        double dx = msg->x - current_goal_.x;
        double dy = msg->y - current_goal_.y;
        double dz = msg->z - current_goal_.z;
        double shift = std::sqrt(dx*dx + dy*dy + dz*dz);

        current_goal_  = *msg;
        goal_received_ = true;

        if (shift > goal_debounce_dist_) {
            goal_changed_ = true;
            RCLCPP_INFO(this->get_logger(),
                "Frontier goal shifted %.1f m -> [%.1f, %.1f, %.1f]",
                shift, msg->x, msg->y, msg->z);
        }
    }

    void stateCallback(const std_msgs::msg::String::SharedPtr msg) {
        active_ = (msg->data == "EXPLORE_CAVE" || msg->data == "EXPLORE");
    }

    // =========================================================================
    // Trajectory visualisation — publish as nav_msgs/Path for RViz
    // =========================================================================

    void publishTrajectoryPath(const Trajectory& traj) {
        nav_msgs::msg::Path path;
        path.header.frame_id = "world";
        path.header.stamp    = this->now();

        constexpr double dt = 0.05;
        for (const auto& seg : traj.segments) {
            for (double t = 0.0; t <= seg.T + 1e-6; t += dt) {
                geometry_msgs::msg::PoseStamped ps;
                ps.header.frame_id = "world";
                ps.header.stamp    = this->now();
                ps.pose.position.x = seg.px.pos(t);
                ps.pose.position.y = seg.py.pos(t);
                ps.pose.position.z = seg.pz.pos(t);
                ps.pose.orientation.w = 1.0;
                path.poses.push_back(ps);
            }
        }
        traj_path_pub_->publish(path);
    }

    // =========================================================================
    // Service handler: ExecuteTrajectory (NAVIGATE_TO_CAVE phase)
    // =========================================================================
    // Builds a multi-segment quintic trajectory through the provided waypoints.
    // Intermediate via-points get non-zero boundary velocities for smooth
    // transitions; endpoints have zero velocity.
    // =========================================================================

    void startNavigationCallback(
        const std::shared_ptr<mission_interfaces::srv::ExecuteTrajectory::Request> request,
        std::shared_ptr<mission_interfaces::srv::ExecuteTrajectory::Response> response)
    {
        if (request->waypoints.poses.size() < 2) {
            response->success = false;
            response->message = "Need at least 2 waypoints";
            return;
        }

        Trajectory traj;
        auto& poses = request->waypoints.poses;

        for (size_t i = 0; i < poses.size() - 1; ++i) {
            Eigen::Vector3d p0(poses[i].position.x,   poses[i].position.y,   poses[i].position.z);
            Eigen::Vector3d pT(poses[i+1].position.x, poses[i+1].position.y, poses[i+1].position.z);
            double dist = (pT - p0).norm();
            double T    = std::max(1.0, dist / navigate_to_cave_speed_);

            // Compute boundary velocities for smooth continuity
            Eigen::Vector3d v0 = Eigen::Vector3d::Zero();
            Eigen::Vector3d vT = Eigen::Vector3d::Zero();
            if (i > 0) {
                Eigen::Vector3d pp(poses[i-1].position.x, poses[i-1].position.y, poses[i-1].position.z);
                v0 = (pT - pp).normalized() * navigate_to_cave_speed_ * 0.5;
            }
            if (i + 2 < poses.size()) {
                Eigen::Vector3d pn(poses[i+2].position.x, poses[i+2].position.y, poses[i+2].position.z);
                vT = (pn - p0).normalized() * navigate_to_cave_speed_ * 0.5;
            }

            TrajSegment seg;
            seg.T  = T;
            seg.px = QuinticPoly::solve(p0.x(), v0.x(), 0, pT.x(), vT.x(), 0, T);
            seg.py = QuinticPoly::solve(p0.y(), v0.y(), 0, pT.y(), vT.y(), 0, T);
            seg.pz = QuinticPoly::solve(p0.z(), v0.z(), 0, pT.z(), vT.z(), 0, T);
            traj.segments.push_back(seg);
        }
        traj.computeTotalTime();

        active_traj_    = traj;
        has_trajectory_ = true;
        finished_pub_   = false;
        traj_start_time_ = this->now();

        response->success = true;
        response->message = "Trajectory started";
        RCLCPP_INFO(this->get_logger(),
            "start_navigation: %zu segments, total=%.2f s", traj.segments.size(), traj.total_time);
    }

    // =========================================================================
    // Collision checking
    // =========================================================================

    // Check a single point against the OctoMap with safety_radius margin
    bool isPointCollisionFree(double x, double y, double z) const {
        if (!octree_) return false;
        auto* node = octree_->search(x, y, z);
        if (node && octree_->isNodeOccupied(node)) return false;

        if (safety_radius_ > 0.0) {
            double r = safety_radius_;
            double offsets[6][3] = {
                {r,0,0}, {-r,0,0}, {0,r,0}, {0,-r,0}, {0,0,r}, {0,0,-r}
            };
            for (auto& off : offsets) {
                auto* n = octree_->search(x + off[0], y + off[1], z + off[2]);
                if (n && octree_->isNodeOccupied(n)) return false;
            }
        }
        return true;
    }

    // Check entire trajectory for collisions at collision_check_dt intervals
    bool isTrajectoryFeasible(const Trajectory& traj) const {
        for (const auto& seg : traj.segments) {
            for (double t = 0.0; t <= seg.T; t += collision_check_dt_) {
                if (!isPointCollisionFree(seg.px.pos(t), seg.py.pos(t), seg.pz.pos(t)))
                    return false;
            }
        }
        return true;
    }

    // Compute minimum clearance along the trajectory (used for scoring).
    // Capped at 5 m to prevent score blowup in open areas.
    double minClearance(const Trajectory& traj) const {
        constexpr double kMax = 5.0;
        double min_clear = kMax;
        for (const auto& seg : traj.segments) {
            for (double t = 0.0; t <= seg.T; t += collision_check_dt_ * 2.0) {
                double x = seg.px.pos(t), y = seg.py.pos(t), z = seg.pz.pos(t);
                for (double r = 0.1; r <= kMax; r += 0.3) {
                    bool occ = false;
                    double offs[6][3] = {{r,0,0},{-r,0,0},{0,r,0},{0,-r,0},{0,0,r},{0,0,-r}};
                    for (auto& off : offs) {
                        auto* n = octree_->search(x + off[0], y + off[1], z + off[2]);
                        if (n && octree_->isNodeOccupied(n)) { occ = true; break; }
                    }
                    if (occ) { min_clear = std::min(min_clear, r); break; }
                }
            }
        }
        return min_clear;
    }

    // =========================================================================
    // Core sampling logic
    // =========================================================================

    // Compute deterministic grid dimensions from a sample budget.
    // Returns (n_lateral, n_vertical) with a 5:3 aspect preference.
    std::pair<int,int> gridDims(int budget, int n_dur) const {
        int spatial = std::max(4, budget / n_dur);
        int n_z   = std::max(2, static_cast<int>(std::round(std::sqrt(spatial * 3.0 / 5.0))));
        int n_lat = std::max(3, spatial / n_z);
        return {n_lat, n_z};
    }

    // Sample candidate trajectories from p0 (with velocity v0) to pGoal.
    // If no direct trajectory is feasible, recursively split at midpoints.
    // Returns true and fills best_traj if a feasible trajectory is found.
    bool sampleTrajectories(const Eigen::Vector3d& p0, const Eigen::Vector3d& v0,
                            const Eigen::Vector3d& pGoal,
                            Trajectory& best_traj, int depth = 0)
    {
        double dist = (pGoal - p0).norm();
        if (dist < 0.5) return false;

        // Terminal velocity: maintain exploration speed toward the goal
        Eigen::Vector3d goal_dir = (pGoal - p0).normalized();
        Eigen::Vector3d vT = goal_dir * (cave_exploration_speed_ * terminal_speed_frac_);
        double nominal_T = dist / cave_exploration_speed_;

        // Build a local coordinate frame aligned with the start→goal direction
        Eigen::Vector3d x_dir = goal_dir;
        Eigen::Vector3d arbitrary = (std::abs(x_dir.z()) < 0.9) ?
            Eigen::Vector3d(0, 0, 1) : Eigen::Vector3d(1, 0, 0);
        Eigen::Vector3d y_dir = x_dir.cross(arbitrary).normalized();
        Eigen::Vector3d z_dir = x_dir.cross(y_dir).normalized();

        // Deterministic stratified grid: n_lat × n_z × n_dur candidates
        constexpr int n_dur = 2;
        auto [n_lat, n_z] = gridDims(num_samples_, n_dur);

        double lat_step = (n_lat > 1) ? (2.0 * lateral_spread_) / (n_lat - 1) : 0.0;
        double z_step   = (n_z > 1)   ? lateral_spread_ / (n_z - 1) : 0.0;

        double T_min = std::max(0.5, nominal_T * min_dur_factor_);
        double T_max = std::max(T_min + 0.1, nominal_T * max_dur_factor_);
        double dur_vals[n_dur] = {T_min, T_max};

        std::vector<Trajectory> candidates;
        candidates.reserve(n_lat * n_z * n_dur);
        int total_checked = 0;

        for (int id = 0; id < n_dur; ++id) {
            double T = dur_vals[id];
            for (int il = 0; il < n_lat; ++il) {
                double lat = -lateral_spread_ + il * lat_step;
                for (int iz = 0; iz < n_z; ++iz) {
                    double z_off = -lateral_spread_ * 0.5 + iz * z_step;

                    Eigen::Vector3d pEnd = pGoal + y_dir * lat + z_dir * z_off;

                    TrajSegment seg;
                    seg.T  = T;
                    seg.px = QuinticPoly::solve(p0.x(), v0.x(), 0, pEnd.x(), vT.x(), 0, T);
                    seg.py = QuinticPoly::solve(p0.y(), v0.y(), 0, pEnd.y(), vT.y(), 0, T);
                    seg.pz = QuinticPoly::solve(p0.z(), v0.z(), 0, pEnd.z(), vT.z(), 0, T);

                    Trajectory traj;
                    traj.segments.push_back(seg);
                    traj.computeTotalTime();
                    ++total_checked;

                    if (isTrajectoryFeasible(traj)) {
                        double clearance = minClearance(traj);
                        double goal_dev  = (y_dir * lat + z_dir * z_off).norm();
                        traj.score = T - 0.5 * clearance + 0.3 * goal_dev;
                        candidates.push_back(traj);
                    }
                }
            }
        }

        // Pick the lowest-score (best) feasible candidate
        if (!candidates.empty()) {
            std::sort(candidates.begin(), candidates.end(),
                [](const Trajectory& a, const Trajectory& b) { return a.score < b.score; });
            best_traj = candidates[0];
            RCLCPP_INFO(this->get_logger(),
                "%zu/%d feasible (best score=%.2f, T=%.2f s) [grid %dx%dx%d]",
                candidates.size(), total_checked, best_traj.score, best_traj.total_time,
                n_lat, n_z, n_dur);
            return true;
        }

        // No single-segment solution → try recursive splitting
        if (depth >= max_recursion_) {
            RCLCPP_WARN(this->get_logger(),
                "Max recursion depth %d reached, no feasible trajectory.", depth);
            return false;
        }

        RCLCPP_INFO(this->get_logger(),
            "No direct trajectory feasible. Splitting (depth=%d)...", depth);

        Eigen::Vector3d midpoint   = (p0 + pGoal) * 0.5;
        double mid_spread          = lateral_spread_ * 1.5;
        auto [mn_lat, mn_z]        = gridDims(num_samples_ / 2, 1);
        double m_lat_step = (mn_lat > 1) ? (2.0 * mid_spread) / (mn_lat - 1) : 0.0;
        double m_z_step   = (mn_z > 1)   ? mid_spread / (mn_z - 1) : 0.0;

        for (int il = 0; il < mn_lat; ++il) {
            double lat = -mid_spread + il * m_lat_step;
            for (int iz = 0; iz < mn_z; ++iz) {
                double z_off = -mid_spread * 0.5 + iz * m_z_step;
                Eigen::Vector3d via = midpoint + y_dir * lat + z_dir * z_off;

                if (!isPointCollisionFree(via.x(), via.y(), via.z())) continue;

                Trajectory traj_first;
                if (!sampleTrajectories(p0, v0, via, traj_first, depth + 1)) continue;

                // Extract terminal velocity of first half for continuity
                auto& last_seg = traj_first.segments.back();
                double T1 = last_seg.T;
                Eigen::Vector3d v_via(last_seg.px.vel(T1), last_seg.py.vel(T1), last_seg.pz.vel(T1));

                Trajectory traj_second;
                if (!sampleTrajectories(via, v_via, pGoal, traj_second, depth + 1)) continue;

                // Concatenate both halves
                Trajectory combined;
                combined.segments.insert(combined.segments.end(),
                    traj_first.segments.begin(), traj_first.segments.end());
                combined.segments.insert(combined.segments.end(),
                    traj_second.segments.begin(), traj_second.segments.end());
                combined.computeTotalTime();
                combined.score = traj_first.score + traj_second.score;

                best_traj = combined;
                RCLCPP_INFO(this->get_logger(),
                    "Multi-segment trajectory: %zu segments, %.2f s total",
                    combined.segments.size(), combined.total_time);
                return true;
            }
        }
        return false;
    }

    // =========================================================================
    // Planning timer — periodically replan during EXPLORE_CAVE
    // =========================================================================

    void planTimerCallback() {
        if (!active_ || !pose_received_ || !goal_received_ || !octree_received_) return;

        Eigen::Vector3d p0(current_pose_.pose.position.x,
                           current_pose_.pose.position.y,
                           current_pose_.pose.position.z);
        Eigen::Vector3d pGoal(current_goal_.x, current_goal_.y, current_goal_.z);

        double dist_to_goal = (pGoal - p0).norm();

        // Skip goals that are too close
        if (dist_to_goal < min_goal_distance_) return;

        bool near_goal = (dist_to_goal < lookahead_distance_) && (dist_to_goal > min_goal_distance_);
        bool need_replan = !has_trajectory_ || finished_pub_ || goal_changed_ || near_goal;

        if (!need_replan) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "Tracking trajectory — goal_dist=%.1f m, remain=%.1f s",
                dist_to_goal, active_traj_.total_time - (this->now() - traj_start_time_).seconds());
            return;
        }

        const char* reason = !has_trajectory_ ? "no_traj" :
                             finished_pub_    ? "traj_done" :
                             goal_changed_    ? "goal_changed" :
                             near_goal        ? "near_goal" : "unknown";

        RCLCPP_INFO(this->get_logger(),
            "Replanning (%s) — pos=[%.1f,%.1f,%.1f] goal=[%.1f,%.1f,%.1f] dist=%.1f m",
            reason, p0.x(), p0.y(), p0.z(), pGoal.x(), pGoal.y(), pGoal.z(), dist_to_goal);

        Eigen::Vector3d v0 = odom_received_ ? current_velocity_ : Eigen::Vector3d::Zero();

        Trajectory best;
        if (sampleTrajectories(p0, v0, pGoal, best)) {
            active_traj_    = best;
            has_trajectory_ = true;
            finished_pub_   = false;
            goal_changed_   = false;
            traj_start_time_ = this->now();
            publishTrajectoryPath(best);
        } else {
            RCLCPP_WARN(this->get_logger(), "No feasible trajectory to goal (dist=%.1f m)", dist_to_goal);
        }
    }

    // =========================================================================
    // Trajectory execution timer (20 Hz) — evaluate the active trajectory
    // =========================================================================

    void trajTimerCallback() {
        if (!has_trajectory_) return;

        double t_elapsed = (this->now() - traj_start_time_).seconds();

        // Determine current segment and local time within that segment
        double cum_time = 0.0;
        size_t seg_idx  = 0;
        double t_local  = 0.0;
        bool   finished = false;

        if (t_elapsed >= active_traj_.total_time) {
            seg_idx  = active_traj_.segments.size() - 1;
            t_local  = active_traj_.segments[seg_idx].T;
            finished = true;
        } else {
            for (size_t i = 0; i < active_traj_.segments.size(); ++i) {
                if (t_elapsed < cum_time + active_traj_.segments[i].T) {
                    seg_idx = i;
                    t_local = t_elapsed - cum_time;
                    break;
                }
                cum_time += active_traj_.segments[i].T;
            }
        }

        const auto& seg = active_traj_.segments[seg_idx];

        // Evaluate quintic polynomials for position, velocity, acceleration
        double px = seg.px.pos(t_local), py = seg.py.pos(t_local), pz = seg.pz.pos(t_local);
        double vx = seg.px.vel(t_local), vy = seg.py.vel(t_local), vz = seg.pz.vel(t_local);
        double ax = seg.px.acc(t_local), ay = seg.py.acc(t_local), az = seg.pz.acc(t_local);

        // Build and publish the desired_state message
        trajectory_msgs::msg::MultiDOFJointTrajectoryPoint msg;
        msg.transforms.resize(1);
        msg.velocities.resize(1);
        msg.accelerations.resize(1);

        msg.transforms[0].translation.x = px;
        msg.transforms[0].translation.y = py;
        msg.transforms[0].translation.z = pz;

        // Yaw follows the horizontal velocity direction; freeze when near-zero speed
        double speed_xy = std::sqrt(vx*vx + vy*vy);
        if (speed_xy > 0.3) last_yaw_ = std::atan2(vy, vx);

        tf2::Quaternion q;
        q.setRPY(0, 0, last_yaw_);
        msg.transforms[0].rotation = tf2::toMsg(q);

        msg.velocities[0].linear.x = vx;
        msg.velocities[0].linear.y = vy;
        msg.velocities[0].linear.z = vz;

        msg.accelerations[0].linear.x = ax;
        msg.accelerations[0].linear.y = ay;
        msg.accelerations[0].linear.z = az;

        desired_pub_->publish(msg);

        // Signal trajectory completion
        if (finished && !finished_pub_) {
            complete_pub_->publish(std_msgs::msg::Empty());
            finished_pub_   = true;
            has_trajectory_ = false;
            RCLCPP_INFO(this->get_logger(),
                "Trajectory complete — end=[%.1f,%.1f,%.1f] speed=%.1f m/s",
                px, py, pz, std::sqrt(vx*vx + vy*vy + vz*vz));
        }
    }
};

// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SamplingPlannerNode>());
    rclcpp::shutdown();
    return 0;
}
