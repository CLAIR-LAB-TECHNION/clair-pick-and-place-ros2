#pragma once

#include <moveit/planning_request_adapter/planning_request_adapter.h>
#include <moveit/planning_interface/planning_interface.h>

namespace ros2srrc_planning_request_adapters
{
class SetDefaultPlannerId : public planning_request_adapter::PlanningRequestAdapter
{
public:
  void initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace) override;

  std::string getDescription() const override;

  bool adaptAndPlan(const PlannerFn& planner, const planning_scene::PlanningSceneConstPtr& planning_scene,
                    const planning_interface::MotionPlanRequest& req,
                    planning_interface::MotionPlanResponse& res,
                    std::vector<std::size_t>& added_path_index) const override;

private:
  std::string default_planner_id_{ "PTP" };
  rclcpp::Logger logger_{ rclcpp::get_logger("SetDefaultPlannerId") };
};

}  // namespace ros2srrc_planning_request_adapters
