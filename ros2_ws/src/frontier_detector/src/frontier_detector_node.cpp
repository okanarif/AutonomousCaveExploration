// =============================================================================
// 3D Frontier Detector for Cave Exploration
// =============================================================================
//
// Detects frontier cells (free voxels adjacent to unknown space) in an OctoMap
// and publishes the best exploration goal for the trajectory planner.
//
// Pipeline:
//   1. Parse OctoMap → extract raw frontier voxels
//   2. Distance pre-filter → keep only frontiers within [min, max] range
//   3. Mean-shift clustering → merge nearby frontiers into cluster centers
//   4. Sort & filter → deduplicate, check occupied neighbors, cave-entrance guard
//   5. Score & select → pick the best frontier based on distance, density, yaw
//   6. Publish goal + RViz markers
//
// The node only runs during the EXPLORE_CAVE state. On first entry into that
// state it resets the OctoMap so exploration starts with a clean map.
// =============================================================================

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <algorithm>
#include <cmath>
#include <memory>
#include <random>
#include <vector>

using namespace std::chrono_literals;

struct Point3D {
    double x, y, z;
};

struct Frontier {
    Point3D coordinates;
    int neighborcount = 0;
    double score = 0.0;
    bool isReachable = false;
};

class FrontierDetector : public rclcpp::Node {
public:
    FrontierDetector() : Node("frontier_detector") {
        // ---- Declare parameters with defaults (overridden by YAML config) ----
        this->declare_parameter("neighborcount_threshold", 100);
        this->declare_parameter("bandwidth", 17.0);
        this->declare_parameter("k_distance", 1.0);
        this->declare_parameter("k_neighborcount", 0.1);
        this->declare_parameter("k_yaw", 55.0);
        this->declare_parameter("distance_limit", 600.0);
        this->declare_parameter("publish_goal_frequency", 2.0);
        this->declare_parameter("occ_neighbor_threshold", 1);
        this->declare_parameter("max_frontiers", 3000);
        this->declare_parameter("min_frontier_distance", 10.0);
        this->declare_parameter("max_frontier_distance", 60.0);

        // Cave entrance coordinates (used to avoid returning to entrance)
        this->declare_parameter("cave_entry_x", -321.0);
        this->declare_parameter("cave_entry_y", 10.0);
        this->declare_parameter("cave_entry_z", 15.0);
        this->declare_parameter("cave_entry_tolerance", 25);

        // Topic names
        this->declare_parameter<std::string>("octomap_topic", "octomap_binary");
        this->declare_parameter<std::string>("pose_topic", "pose_est");
        this->declare_parameter<std::string>("state_topic", "stm_mode");
        this->declare_parameter<std::string>("frontiers_marker_topic", "frontiers");
        this->declare_parameter<std::string>("goal_pose_topic", "frontier_goal_pose");
        this->declare_parameter<std::string>("goal_point_topic", "frontier_goal");
        this->declare_parameter<std::string>("octomap_reset_service", "octomap_server/reset");
        this->declare_parameter<std::string>("start_exploration_service", "start_exploration");

        // Load parameters
        neighborcount_threshold_ = this->get_parameter("neighborcount_threshold").as_int();
        bandwidth_               = this->get_parameter("bandwidth").as_double();
        k_distance_              = this->get_parameter("k_distance").as_double();
        k_neighborcount_         = this->get_parameter("k_neighborcount").as_double();
        k_yaw_                   = this->get_parameter("k_yaw").as_double();
        distance_limit_          = this->get_parameter("distance_limit").as_double();
        publish_goal_frequency_  = this->get_parameter("publish_goal_frequency").as_double();
        occ_neighbor_threshold_  = this->get_parameter("occ_neighbor_threshold").as_int();
        max_frontiers_           = this->get_parameter("max_frontiers").as_int();
        min_frontier_distance_   = this->get_parameter("min_frontier_distance").as_double();
        max_frontier_distance_   = this->get_parameter("max_frontier_distance").as_double();

        cave_entry_point_.x  = this->get_parameter("cave_entry_x").as_double();
        cave_entry_point_.y  = this->get_parameter("cave_entry_y").as_double();
        cave_entry_point_.z  = this->get_parameter("cave_entry_z").as_double();
        cave_entry_tolerance_ = this->get_parameter("cave_entry_tolerance").as_int();

        std::string octomap_topic       = this->get_parameter("octomap_topic").as_string();
        std::string pose_topic          = this->get_parameter("pose_topic").as_string();
        std::string state_topic         = this->get_parameter("state_topic").as_string();
        std::string frontiers_topic     = this->get_parameter("frontiers_marker_topic").as_string();
        std::string goal_pose_topic     = this->get_parameter("goal_pose_topic").as_string();
        std::string goal_point_topic    = this->get_parameter("goal_point_topic").as_string();
        std::string reset_service       = this->get_parameter("octomap_reset_service").as_string();
        std::string exploration_service = this->get_parameter("start_exploration_service").as_string();

        // ---- Subscribers ----
        auto octomap_qos = rclcpp::QoS(1).transient_local().reliable();
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic, octomap_qos,
            std::bind(&FrontierDetector::parseOctomap, this, std::placeholders::_1));

        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic, 1,
            std::bind(&FrontierDetector::onPose, this, std::placeholders::_1));

        state_sub_ = this->create_subscription<std_msgs::msg::String>(
            state_topic, 1,
            std::bind(&FrontierDetector::onState, this, std::placeholders::_1));

        // ---- Publishers ----
        frontier_pub_   = this->create_publisher<visualization_msgs::msg::MarkerArray>(frontiers_topic, 1);
        goal_pub_       = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_pose_topic, 1);
        goal_point_pub_ = this->create_publisher<geometry_msgs::msg::Point>(goal_point_topic, 1);

        // ---- Service client (OctoMap reset) ----
        octomap_reset_client_ = this->create_client<std_srvs::srv::Empty>(reset_service);

        // ---- Service server (mission_control compatibility) ----
        start_exploration_srv_ = this->create_service<std_srvs::srv::Trigger>(
            exploration_service,
            std::bind(&FrontierDetector::startExplorationCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---- Goal publish timer ----
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / publish_goal_frequency_),
            std::bind(&FrontierDetector::publishGoal, this));

        RCLCPP_INFO(this->get_logger(), "Frontier detector initialized");
    }

private:
    // ---- Parameters ----
    int    neighborcount_threshold_;
    double bandwidth_;
    double k_distance_;
    double k_neighborcount_;
    double k_yaw_;
    double distance_limit_;
    double publish_goal_frequency_;
    int    occ_neighbor_threshold_;
    int    max_frontiers_;
    double min_frontier_distance_;
    double max_frontier_distance_;
    int    cave_entry_tolerance_;

    // ---- ROS interfaces ----
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr    octomap_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr          state_sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr frontier_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr   goal_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr         goal_point_pub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr                octomap_reset_client_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr             start_exploration_srv_;
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- Internal state ----
    Point3D     curr_drone_position_ = {0.0, 0.0, 0.0};
    double      drone_yaw_           = 0.0;
    std::string statemachine_state_  = "IDLE";
    bool        octomap_reset_done_  = false;
    float       octomap_res_         = 0.5f;
    bool        waiting_for_reset_   = false;  // True while waiting for fresh post-reset OctoMap

    // Current best goal
    geometry_msgs::msg::PoseStamped goal_message_;
    geometry_msgs::msg::Point       goal_point_;
    bool goal_available_ = false;

    Point3D cave_entry_point_;

    // =========================================================================
    // State machine callback – resets OctoMap once on first EXPLORE entry
    // =========================================================================
    void onState(const std_msgs::msg::String::SharedPtr msg) {
        std::string prev = statemachine_state_;
        statemachine_state_ = msg->data;

        bool entering_explore = (statemachine_state_ == "EXPLORE_CAVE" ||
                                 statemachine_state_ == "EXPLORE");
        bool was_not_exploring = (prev != "EXPLORE_CAVE" && prev != "EXPLORE");

        if (entering_explore && was_not_exploring && !octomap_reset_done_) {
            RCLCPP_INFO(this->get_logger(), "Entering exploration. Resetting OctoMap.");
            resetOctomap();
            octomap_reset_done_ = true;
        }
    }

    // =========================================================================
    // start_exploration service (mission_control compatibility)
    // =========================================================================
    void startExplorationCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res)
    {
        res->success = true;
        res->message = "Exploration controlled by state machine";
    }

    // =========================================================================
    // Pose callback
    // =========================================================================
    void onPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        curr_drone_position_.x = msg->pose.position.x;
        curr_drone_position_.y = msg->pose.position.y;
        curr_drone_position_.z = msg->pose.position.z;

        tf2::Quaternion quat(
            msg->pose.orientation.x,
            msg->pose.orientation.y,
            msg->pose.orientation.z,
            msg->pose.orientation.w);
        tf2::Matrix3x3 mat(quat);
        double roll, pitch;
        mat.getRPY(roll, pitch, drone_yaw_);
    }

    // =========================================================================
    // OctoMap reset (calls octomap_server/reset service)
    // =========================================================================
    void resetOctomap() {
        if (!octomap_reset_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "OctoMap reset service not available, skipping.");
            return;
        }
        waiting_for_reset_ = true;
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        octomap_reset_client_->async_send_request(request);
    }

    // =========================================================================
    // Utility functions
    // =========================================================================
    double euclideanDistance(const Point3D& p1, const Point3D& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        double dz = p1.z - p2.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    double gaussianKernel(double distance) {
        return std::exp(-(distance * distance) / (2.0 * bandwidth_ * bandwidth_));
    }

    // =========================================================================
    // Mean-shift clustering
    // Iteratively shifts each point toward the local density maximum using a
    // Gaussian kernel until all points converge (shift < threshold).
    // =========================================================================
    std::vector<Point3D> meanShiftClustering(
        const std::vector<Frontier>& points, double convergenceThreshold)
    {
        std::vector<Point3D> shiftedPoints;
        shiftedPoints.reserve(points.size());
        for (const auto& f : points) {
            shiftedPoints.push_back(f.coordinates);
        }

        bool converged = false;
        while (!converged) {
            converged = true;
            for (size_t i = 0; i < shiftedPoints.size(); ++i) {
                Point3D original = shiftedPoints[i];
                Point3D shifted  = {0.0, 0.0, 0.0};
                double  totalW   = 0.0;

                for (const auto& p : points) {
                    double dist   = euclideanDistance(original, p.coordinates);
                    double weight = gaussianKernel(dist);
                    shifted.x += p.coordinates.x * weight;
                    shifted.y += p.coordinates.y * weight;
                    shifted.z += p.coordinates.z * weight;
                    totalW    += weight;
                }

                if (totalW > 0.0) {
                    shifted.x /= totalW;
                    shifted.y /= totalW;
                    shifted.z /= totalW;
                }

                if (euclideanDistance(original, shifted) > convergenceThreshold) {
                    shiftedPoints[i] = shifted;
                    converged = false;
                }
            }
        }

        return shiftedPoints;
    }

    // =========================================================================
    // Frontier scoring function
    // Combines distance penalty, cluster density reward, and yaw alignment
    // penalty into a single score. Higher is better.
    // =========================================================================
    double getScore(const Frontier& frontier) {
        double yaw_to_frontier = std::atan2(
            frontier.coordinates.y - curr_drone_position_.y,
            frontier.coordinates.x - curr_drone_position_.x);
        double yaw_score = std::abs(yaw_to_frontier - drone_yaw_);
        if (yaw_score > M_PI) {
            yaw_score = 2.0 * M_PI - yaw_score;
        }
        return -k_distance_      * euclideanDistance(frontier.coordinates, curr_drone_position_)
               + k_neighborcount_ * static_cast<double>(frontier.neighborcount)
               - k_yaw_           * yaw_score;
    }

    // =========================================================================
    // Timer callback – publish current best goal while exploring
    // =========================================================================
    void publishGoal() {
        if (statemachine_state_ != "EXPLORE_CAVE" && statemachine_state_ != "EXPLORE") {
            return;
        }
        if (!goal_available_) return;

        goal_pub_->publish(goal_message_);
        goal_point_pub_->publish(goal_point_);
    }

    // =========================================================================
    // Build goal message from the selected frontier
    // =========================================================================
    void setGoalMessage(const Frontier& best_frontier) {
        goal_message_.header.frame_id   = "world";
        goal_message_.header.stamp      = this->now();
        goal_message_.pose.position.x   = best_frontier.coordinates.x;
        goal_message_.pose.position.y   = best_frontier.coordinates.y;
        goal_message_.pose.position.z   = best_frontier.coordinates.z;

        double goal_yaw = std::atan2(
            best_frontier.coordinates.y - curr_drone_position_.y,
            best_frontier.coordinates.x - curr_drone_position_.x);
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, goal_yaw);
        goal_message_.pose.orientation.x = q.x();
        goal_message_.pose.orientation.y = q.y();
        goal_message_.pose.orientation.z = q.z();
        goal_message_.pose.orientation.w = q.w();

        goal_point_.x = best_frontier.coordinates.x;
        goal_point_.y = best_frontier.coordinates.y;
        goal_point_.z = best_frontier.coordinates.z;

        goal_available_ = true;
    }

    // =========================================================================
    // Select the highest-scoring frontier and set it as the goal
    // =========================================================================
    void selectFrontier(const std::vector<Frontier>& points_sorted) {
        if (points_sorted.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                "No frontier candidates after filtering");
            return;
        }

        Frontier best = points_sorted[0];
        best.score = getScore(best);

        for (const auto& f : points_sorted) {
            Frontier candidate = f;
            candidate.score = getScore(candidate);
            if (candidate.score > best.score) {
                best = candidate;
            }
        }

        if (best.isReachable &&
            euclideanDistance(curr_drone_position_, best.coordinates) < distance_limit_)
        {
            setGoalMessage(best);
        }
    }

    // =========================================================================
    // Publish frontier markers for RViz visualization
    // =========================================================================
    void publishMarkers(const std::vector<Frontier>& points_sorted) {
        visualization_msgs::msg::MarkerArray markerArray;

        // Clear previous markers
        visualization_msgs::msg::Marker clear;
        clear.id     = 0;
        clear.ns     = "frontier";
        clear.action = visualization_msgs::msg::Marker::DELETEALL;
        markerArray.markers.push_back(clear);

        for (int i = 0; i < static_cast<int>(points_sorted.size()); ++i) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "world";
            m.header.stamp    = this->now();
            m.ns              = "frontier";
            m.id              = i + 1;
            m.type            = visualization_msgs::msg::Marker::CUBE;
            m.action          = visualization_msgs::msg::Marker::ADD;
            m.pose.position.x = points_sorted[i].coordinates.x;
            m.pose.position.y = points_sorted[i].coordinates.y;
            m.pose.position.z = points_sorted[i].coordinates.z;
            m.pose.orientation.w = 1.0;
            m.scale.x         = octomap_res_;
            m.scale.y         = octomap_res_;
            m.scale.z         = octomap_res_;
            m.color.r         = 1.0f;
            m.color.g         = 0.0f;
            m.color.b         = 0.0f;
            m.color.a         = 1.0f;
            markerArray.markers.push_back(m);
        }

        frontier_pub_->publish(markerArray);
    }

    // =========================================================================
    // Filter and deduplicate clustered frontiers
    //   - Compute cluster density (neighbor count within one voxel)
    //   - Deduplicate points closer than one voxel
    //   - Reject frontiers with too many occupied corner-neighbors
    //   - Reject frontiers that would lead back toward the cave entrance
    //   - Keep only frontiers within the configured distance range
    // =========================================================================
    std::vector<Frontier> sortFrontiers(
        const std::vector<Point3D>& frontiers_clustered,
        octomap::OcTree* octree)
    {
        std::vector<Frontier> frontiers_sorted;

        for (const auto& f1 : frontiers_clustered) {
            Frontier frontier;
            frontier.coordinates   = f1;
            frontier.neighborcount = 0;

            // Cluster density: count clustered points within one voxel
            for (const auto& f2 : frontiers_clustered) {
                if (euclideanDistance(f1, f2) < static_cast<double>(octomap_res_)) {
                    frontier.neighborcount++;
                }
            }

            // Deduplication: skip if a frontier already exists within one voxel
            bool unique = true;
            for (const auto& f3 : frontiers_sorted) {
                if (euclideanDistance(f3.coordinates, frontier.coordinates) <
                    static_cast<double>(octomap_res_))
                {
                    unique = false;
                    break;
                }
            }

            // Count occupied corner-neighbors (8 diagonal corners)
            int occ_corners = 0;
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dz = -1; dz <= 1; ++dz) {
                        if (dx != 0 && dy != 0 && dz != 0) {
                            octomap::point3d pt(
                                frontier.coordinates.x + octomap_res_ * dx,
                                frontier.coordinates.y + octomap_res_ * dy,
                                frontier.coordinates.z + octomap_res_ * dz);
                            octomap::OcTreeNode* result = octree->search(pt);
                            if (result != nullptr && octree->isNodeOccupied(result)) {
                                occ_corners++;
                            }
                        }
                    }
                }
            }

            // Cave entrance guard: once the drone is past the entrance, reject
            // frontiers that would send it back toward the entrance
            bool not_toward_entrance = true;
            if (curr_drone_position_.x < cave_entry_point_.x - cave_entry_tolerance_ &&
                frontier.coordinates.x  > cave_entry_point_.x - cave_entry_tolerance_)
            {
                not_toward_entrance = false;
            }

            double dist_to_drone = euclideanDistance(frontier.coordinates, curr_drone_position_);

            if (unique &&
                frontier.neighborcount > neighborcount_threshold_ &&
                not_toward_entrance &&
                occ_corners < occ_neighbor_threshold_ &&
                dist_to_drone >= min_frontier_distance_ &&
                dist_to_drone <= max_frontier_distance_)
            {
                frontier.isReachable = true;
                frontiers_sorted.push_back(frontier);
            }
        }

        return frontiers_sorted;
    }

    // =========================================================================
    // Main OctoMap callback – full frontier detection pipeline
    // Only processes during EXPLORE_CAVE state.
    // =========================================================================
    void parseOctomap(const octomap_msgs::msg::Octomap::SharedPtr msg) {
        if (statemachine_state_ != "EXPLORE_CAVE" && statemachine_state_ != "EXPLORE") {
            return;
        }

        // Skip stale pre-reset messages until a fresh (small) map arrives
        if (waiting_for_reset_) {
            octomap::AbstractOcTree* probe = octomap_msgs::msgToMap(*msg);
            octomap::OcTree* probe_tree = dynamic_cast<octomap::OcTree*>(probe);
            size_t leaf_count = probe_tree ? probe_tree->getNumLeafNodes() : 0;
            if (probe_tree) delete probe_tree;

            if (leaf_count > 100) {
                return;  // Still the stale pre-reset map
            }
            waiting_for_reset_ = false;
            RCLCPP_INFO(this->get_logger(), "Post-reset OctoMap received. Resuming frontier detection.");
        }

        octomap::AbstractOcTree* tree = octomap_msgs::msgToMap(*msg);
        octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(tree);

        if (!octree) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse OctoMap message");
            if (tree) delete tree;
            return;
        }

        octomap_res_ = static_cast<float>(octree->getResolution());

        // Step 1: Extract raw frontiers (free voxels with unknown diagonal neighbors)
        std::vector<Frontier> frontiers;

        for (auto it = octree->begin_leafs(); it != octree->end_leafs(); ++it) {
            if (!octree->isNodeOccupied(*it)) {
                for (int dx = -1; dx <= 1; ++dx) {
                    for (int dy = -1; dy <= 1; ++dy) {
                        for (int dz = -1; dz <= 1; ++dz) {
                            if (dx != 0 && dy != 0 && dz != 0) {
                                octomap::OcTreeKey neighborKey(
                                    it.getKey().k[0] + dx,
                                    it.getKey().k[1] + dy,
                                    it.getKey().k[2] + dz);
                                if (octree->search(neighborKey) == nullptr) {
                                    Frontier fp;
                                    fp.coordinates.x = it.getX();
                                    fp.coordinates.y = it.getY();
                                    fp.coordinates.z = it.getZ();
                                    frontiers.push_back(fp);
                                }
                            }
                        }
                    }
                }
            }
        }

        if (frontiers.empty()) {
            delete octree;
            return;
        }

        // Step 2: Distance pre-filter – keep only frontiers within operating range
        {
            std::vector<Frontier> in_range;
            in_range.reserve(frontiers.size());
            for (const auto& f : frontiers) {
                double d = euclideanDistance(f.coordinates, curr_drone_position_);
                if (d >= min_frontier_distance_ && d <= max_frontier_distance_) {
                    in_range.push_back(f);
                }
            }
            frontiers = std::move(in_range);
        }

        if (frontiers.empty()) {
            delete octree;
            return;
        }

        // Step 3: Random subsample to cap mean-shift O(n^2) cost
        if (static_cast<int>(frontiers.size()) > max_frontiers_) {
            std::mt19937 rng(42);
            std::shuffle(frontiers.begin(), frontiers.end(), rng);
            frontiers.resize(max_frontiers_);
        }

        // Step 4: Mean-shift clustering
        std::vector<Point3D> frontierpoints_clustered =
            meanShiftClustering(frontiers, 1.0);

        // Step 5: Filter, deduplicate, and score
        std::vector<Frontier> frontierpoints_sorted =
            sortFrontiers(frontierpoints_clustered, octree);

        // Step 6: Select best goal and publish markers
        selectFrontier(frontierpoints_sorted);
        publishMarkers(frontierpoints_sorted);

        delete octree;
    }
};

// =============================================================================
// Entry point
// =============================================================================
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierDetector>());
    rclcpp::shutdown();
    return 0;
}
