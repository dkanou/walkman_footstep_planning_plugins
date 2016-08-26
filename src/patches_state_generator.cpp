#include <walkman_footstep_planning_plugins/patches_state_generator.h>

#include <vigir_footstep_planning_plugins/plugin_aggregators/robot_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/world_model.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/step_cost_estimator.h>
#include <vigir_footstep_planning_plugins/plugin_aggregators/post_processor.h>



namespace walkman_footstep_planning
{
using namespace vigir_footstep_planning;

PatchesStateGenerator::PatchesStateGenerator()
: StateGeneratorPlugin("patches_state_generator")
{}

bool PatchesStateGenerator::initialize(const vigir_generic_params::ParameterSet& global_params)
{
  return StateGeneratorPlugin::initialize(global_params);
}

bool PatchesStateGenerator::postInitialize(const vigir_generic_params::ParameterSet& global_params)
{
  return StateGeneratorPlugin::postInitialize(global_params);
}

bool PatchesStateGenerator::loadParams(const vigir_generic_params::ParameterSet& global_params)
{
  return StateGeneratorPlugin::loadParams(global_params);
}

void PatchesStateGenerator::reset()
{
  StateGeneratorPlugin::reset();
}

std::list<PlanningState::Ptr> PatchesStateGenerator::generatePredecessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  ROS_ERROR("[PatchesStateGenerator] generatePredecessor not implemented yet!");

  return result;
}

std::list<PlanningState::Ptr> PatchesStateGenerator::generateSuccessor(const PlanningState& state) const
{
  std::list<PlanningState::Ptr> result;

  /// TODO: Fill result list based on available patches in reasonable region around the current state

  for (std::list<PlanningState::Ptr>::iterator itr = result.begin(); itr != result.end();)
  {
    if (!(*itr))
    {
      result.erase(itr++);
      continue;
    }

    State& new_state = (*itr)->getState();

    const State& left_foot = state.getLeg() == LEFT ? state.getState() : state.getPredState()->getState();
    const State& right_foot = state.getLeg() == RIGHT ? state.getState() : state.getPredState()->getState();

    // update 3D pose based on world data
    WorldModel::instance().update3DData(new_state);

    // apply post processing steps
    PostProcessor::instance().postProcessForward(left_foot, right_foot, new_state);

    // check if new state is reachable for robot
    if (!RobotModel::instance().isReachable(left_foot, right_foot, new_state))
    {
      result.erase(itr++);
      continue;
    }

    // collision check
    if (!WorldModel::instance().isAccessible(new_state, state.getState()))
    {
      result.erase(itr++);
      continue;
    }

    // lookup costs
    double cost, risk;
    if (!StepCostEstimator::instance().getCost(left_foot, right_foot, new_state, cost, risk))
    {
      result.erase(itr++);
      continue;
    }

    new_state.setCost(cost);
    new_state.setRisk(risk);

    itr++;
  }

  return result;
}
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(walkman_footstep_planning::PatchesStateGenerator, vigir_footstep_planning::StateGeneratorPlugin)
