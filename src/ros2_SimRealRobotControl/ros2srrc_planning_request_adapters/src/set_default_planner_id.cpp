#include <pluginlib/class_list_macros.hpp>
#include <ros2srrc_planning_request_adapters/set_default_planner_id.hpp>

namespace ros2srrc_planning_request_adapters
{
void SetDefaultPlannerId::initialize(const rclcpp::Node::SharedPtr& node, const std::string& parameter_namespace)
{
  std::string param_name = parameter_namespace.empty() ? "default_planner_id" : parameter_namespace + ".default_planner_id";
  node->declare_parameter<std::string>(param_name, default_planner_id_);
  node->get_parameter(param_name, default_planner_id_);
  RCLCPP_INFO(logger_, "SetDefaultPlannerId: default_planner_id = '%s'", default_planner_id_.c_str());
}

std::string SetDefaultPlannerId::getDescription() const
{
  return "Sets planner_id to the configured default when the request has an empty planner_id.";
}

bool SetDefaultPlannerId::adaptAndPlan(const PlannerFn& planner,
                                       const planning_scene::PlanningSceneConstPtr& planning_scene,
                                       const planning_interface::MotionPlanRequest& req,
                                       planning_interface::MotionPlanResponse& res,
                                       std::vector<std::size_t>& added_path_index) const
{
  (void)added_path_index;
  planning_interface::MotionPlanRequest modified_req = req;
  if (modified_req.planner_id.empty())
  {
    modified_req.planner_id = default_planner_id_;
    RCLCPP_DEBUG(logger_, "SetDefaultPlannerId: substituted empty planner_id with '%s'", default_planner_id_.c_str());
  }
  return planner(planning_scene, modified_req, res);
}

}  // namespace ros2srrc_planning_request_adapters

PLUGINLIB_EXPORT_CLASS(ros2srrc_planning_request_adapters::SetDefaultPlannerId,
                       planning_request_adapter::PlanningRequestAdapter)
