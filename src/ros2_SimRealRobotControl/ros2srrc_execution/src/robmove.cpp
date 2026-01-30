// RobMove.cpp:

#include <string>

// Required to include ROS2 and ROS2 Action Server:
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

// Include the /Robmove ROS2 Action:
#include "ros2srrc_data/action/robmove.hpp"

// Include MoveIt!2:
#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// Declaration of GLOBAL VARIABLE --> MoveIt!2 Interface:
moveit::planning_interface::MoveGroupInterface move_group_interface_ROB;

// Declaration of GLOBAL VARIABLE --> ROBOT PARAMETER:
std::string param_ROB = "none";

// Declaration of GLOBAL VARIABLE --> RES:
auto RES = "none";

// Last MoveIt planning error code (for detailed failure message):
int last_plan_error_code = 0;

// =============================================================================== //
// Map MoveIt error code to string for diagnostics:
static std::string moveit_error_string(int code) {
    switch (code) {
        case  1: return "SUCCESS";
        case -1: return "PLANNING_FAILED";
        case -2: return "INVALID_MOTION_PLAN";
        case -3: return "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE";
        case -4: return "CONTROL_FAILED";
        case -5: return "UNABLE_TO_AQUIRE_SENSOR_DATA";
        case -6: return "TIMED_OUT";
        case -7: return "PREEMPTED";
        case -10: return "START_STATE_IN_COLLISION";
        case -11: return "START_STATE_VIOLATES_PATH_CONSTRAINTS";
        case -12: return "GOAL_IN_COLLISION";
        case -13: return "GOAL_VIOLATES_PATH_CONSTRAINTS";
        case -14: return "GOAL_CONSTRAINTS_VIOLATED";
        case -15: return "INVALID_GROUP_NAME";
        case -16: return "INVALID_GOAL_CONSTRAINTS";
        case -17: return "INVALID_ROBOT_STATE";
        case -18: return "INVALID_LINK_NAME";
        case -19: return "INVALID_OBJECT_NAME";
        case -21: return "FRAME_TRANSFORM_FAILURE";
        case -22: return "COLLISION_CHECKING_UNAVAILABLE";
        case -23: return "ROBOT_STATE_STALE";
        case -24: return "SENSOR_INFO_STALE";
        case -25: return "COMMUNICATION_FAILURE";
        case -26: return "START_STATE_INVALID";
        case -27: return "GOAL_STATE_INVALID";
        case -28: return "UNRECOGNIZED_GOAL_TYPE";
        case -29: return "CRASH";
        case -30: return "ABORT";
        case -31: return "NO_IK_SOLUTION";
        default: return "FAILURE(code=" + std::to_string(code) + ")";
    }
}

// =============================================================================== //
//  PARAM -> ROBOT:

class ros2_RobotParam : public rclcpp::Node
{
public:
    ros2_RobotParam() : Node("ros2_RobotParam") 
    {
        this->declare_parameter("ROB_PARAM", "none");
        param_ROB = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", param_ROB.c_str());
    }
private:
};

// =============================================================================== //
// Planning configuration constants:
const int MAX_PLANNING_ATTEMPTS = 5;        // Number of planning attempts before giving up
const double PLANNING_TIME_SECONDS = 10.0;  // Planning time per attempt (seconds)

// =============================================================================== //
// MoveIt!2 -> MoveGroupInterface/Plan function with multiple attempts:

moveit::planning_interface::MoveGroupInterface::Plan plan_ROB() {
    
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    
    // Set planning time for each attempt
    move_group_interface_ROB.setPlanningTime(PLANNING_TIME_SECONDS);
    
    bool success = false;
    
    // Try multiple planning attempts
    for (int attempt = 1; attempt <= MAX_PLANNING_ATTEMPTS; attempt++) {
        
        RCLCPP_INFO(rclcpp::get_logger("plan_ROB"), 
                    "Planning attempt %d/%d (%.1fs timeout)...", 
                    attempt, MAX_PLANNING_ATTEMPTS, PLANNING_TIME_SECONDS);
        
        auto plan_result = move_group_interface_ROB.plan(my_plan);
        last_plan_error_code = plan_result.val;
        success = (plan_result == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        
        if (success) {
            RCLCPP_INFO(rclcpp::get_logger("plan_ROB"), 
                        "Planning succeeded on attempt %d/%d", attempt, MAX_PLANNING_ATTEMPTS);
            break;
        } else {
            RCLCPP_WARN(rclcpp::get_logger("plan_ROB"), 
                        "Planning attempt %d/%d failed (MoveIt: %s), %s", 
                        attempt, MAX_PLANNING_ATTEMPTS,
                        moveit_error_string(last_plan_error_code).c_str(),
                        (attempt < MAX_PLANNING_ATTEMPTS) ? "retrying..." : "no more attempts.");
        }
    }

    if (success)
    {
        RES = "PLANNING: OK";
        return(my_plan);
    }
    else
    {
        RES = "PLANNING: ERROR";
        return(my_plan);
    }

};

// =============================================================================== //
// ROS2 Action Server to move the ROBOT:

class ActionServer : public rclcpp::Node
{

public:
    using Robmove = ros2srrc_data::action::Robmove;
    using GoalHandle = rclcpp_action::ServerGoalHandle<Robmove>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("ros2srrc_RobMove", options){

        action_server_ = rclcpp_action::create_server<Robmove>(
            this,
            "/Robmove",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1)
            );

    }

private:
    rclcpp_action::Server<Robmove>::SharedPtr action_server_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const Robmove::Goal> goal)
    {
        RCLCPP_INFO(get_logger(), "RobMove (/Robmove) -> RECEIVED A ROBOT MOVEMENT REQUEST:");
        RCLCPP_INFO(get_logger(), "Movement TYPE -> %s", goal->type.c_str());
        RCLCPP_INFO(get_logger(), "Movement SPEED -> %.2f", goal->speed);
        RCLCPP_INFO(get_logger(), "Desired POSITION -> (x: %.3f, y: %.3f, z: %.3f)", goal->x, goal->y, goal->z);
        RCLCPP_INFO(get_logger(), "DESIRED ORIENTATION -> (qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f)", goal->qx, goal->qy, goal->qz, goal->qw);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");
        move_group_interface_ROB.stop();
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {

        // 0. INFORMATION -> Current Robot Pose:
        auto CP_INFO = move_group_interface_ROB.getCurrentPose();
        RCLCPP_INFO(get_logger(), "INFORMATION -> Current Robot Pose:");
        RCLCPP_INFO(get_logger(), "POSITION -> (x: %.3f, y: %.3f, z: %.3f)", CP_INFO.pose.position.x, CP_INFO.pose.position.y, CP_INFO.pose.position.z);
        RCLCPP_INFO(get_logger(), "ORIENTATION -> (qx: %.3f, qy: %.3f, qz: %.3f, qw: %.3f)", CP_INFO.pose.orientation.x, CP_INFO.pose.orientation.y, CP_INFO.pose.orientation.z, CP_INFO.pose.orientation.w);
        
        // 1. OBTAIN INPUT PARAMETERS:
        const auto GOAL = goal_handle->get_goal();

        // 2. DECLARE RESULT:
        auto RESULT = std::make_shared<Robmove::Result>();

        // 3. Robot Movement -> EXECUTION:

        moveit::planning_interface::MoveGroupInterface::Plan MyPlan;
        
        auto CURRENT_POSE = move_group_interface_ROB.getCurrentPose();

        geometry_msgs::msg::Pose TARGET_POSE;
        TARGET_POSE.position.x = GOAL->x;
        TARGET_POSE.position.y = GOAL->y;
        TARGET_POSE.position.z = GOAL->z;
        TARGET_POSE.orientation.x = GOAL->qx;
        TARGET_POSE.orientation.y = GOAL->qy;
        TARGET_POSE.orientation.z = GOAL->qz;
        TARGET_POSE.orientation.w = GOAL->qw;

        move_group_interface_ROB.setPoseTarget(TARGET_POSE);

        move_group_interface_ROB.setPlannerId(GOAL->type);
        move_group_interface_ROB.setMaxVelocityScalingFactor(GOAL->speed);

        MyPlan = plan_ROB();

        if (RES == "PLANNING: OK"){

            bool ExecSUCCESS = (move_group_interface_ROB.execute(MyPlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) has been CANCELED.", GOAL->type.c_str());
                RESULT->success = false;
                RESULT->message = "RobMove: CANCELED";
                goal_handle->canceled(RESULT);
                return;
            } 
            
            if (ExecSUCCESS){
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) successfully executed.", GOAL->type.c_str());
                RESULT->success = true;
                RESULT->message = "RobMove: SUCCESS";
                goal_handle->succeed(RESULT);
            } else {
                RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) failed. Reason -> EXECUTION failure.", GOAL->type.c_str());
                RESULT->success = false;
                RESULT->message = "RobMove: EXECUTION FAILED";
                goal_handle->succeed(RESULT);
            }

        } else {
            std::string detail = moveit_error_string(last_plan_error_code);
            RCLCPP_INFO(this->get_logger(), "ROBOT MOVEMENT (%s) failed. Reason -> PLANNING failure (MoveIt: %s). Target pose: x=%.3f y=%.3f z=%.3f",
                        GOAL->type.c_str(), detail.c_str(), GOAL->x, GOAL->y, GOAL->z);
            RESULT->success = false;
            RESULT->message = "RobMove: PLANNING FAILED (MoveIt: " + detail + ")";
            goal_handle->succeed(RESULT);
        }

        RES = "none";

    }

};

// ===================================================================================== //
// ======================================= MAIN ======================================== //
// ===================================================================================== //

int main(int argc, char **argv)
{

    // Initialise MAIN NODE:
    rclcpp::init(argc, argv);
    
    auto node_LOGGER = std::make_shared<rclcpp::Node>("MOVE_INTERFACE_log");

    // Obtain ROBOT parameter:
    auto node_PARAM_ROB = std::make_shared<ros2_RobotParam>();
    rclcpp::spin_some(node_PARAM_ROB);

    // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
    auto name = "ros2srrc_RobMove";
    auto const MoveIt2_NODE = std::make_shared<rclcpp::Node>(name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
    rclcpp::executors::SingleThreadedExecutor executor; 
    executor.add_node(MoveIt2_NODE);
    std::thread([&executor]() { executor.spin(); }).detach();

    // MoveGroupInterface_ROB:
    using moveit::planning_interface::MoveGroupInterface;
    auto ROBname = param_ROB + "_arm";
    move_group_interface_ROB = MoveGroupInterface(MoveIt2_NODE, ROBname);
    move_group_interface_ROB.setPlanningPipelineId("move_group");

    move_group_interface_ROB.setMaxVelocityScalingFactor(1.0);
    move_group_interface_ROB.setMaxAccelerationScalingFactor(1.0);
    
    RCLCPP_INFO(node_LOGGER->get_logger(), "MoveGroupInterface object created for ROBOT: %s", param_ROB.c_str());

    // CREATE -> PlanningSceneInterface:
    using moveit::planning_interface::PlanningSceneInterface;
    auto planning_scene_interface = PlanningSceneInterface();

    // Declare and spin ACTION SERVER:
    auto action_server = std::make_shared<ActionServer>();
    rclcpp::spin(action_server);

    rclcpp::shutdown();
    return 0;

}