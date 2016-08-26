//=================================================================================================
// Copyright (c) 2016, Alexander Stumpf, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#ifndef WALKMAN_PATCHES_STATE_GENERATOR_H__
#define WALKMAN_PATCHES_STATE_GENERATOR_H__

#include <vigir_footstep_planning_plugins/plugins/state_generator_plugin.h>



namespace walkman_footstep_planning
{
using namespace vigir_footstep_planning;

class PatchesStateGenerator
  : public StateGeneratorPlugin
{
public:
  PatchesStateGenerator();

  /**
   * @brief Initialization of plugin specific features.
   * @param global_params global parameter set
   * @return true when initialization was successful
   */
  bool initialize(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  /**
   * @brief Called after initialization of this and other plugins has been completed.
   * At this point other plugins can be used safely.
   * @param global_params global parameter set
   * @return true, if post initialization was successful
   */
  bool postInitialize(const vigir_generic_params::ParameterSet& global_params = vigir_generic_params::ParameterSet()) override;

  /**
   * @brief Loads parameters from parameter set and rosparam server (!= plugin's dedicated parameters).
   * This method will be automatically called by Plugin::initialize(...).
   * @param global_params global parameter set
   * @return true, if loading parameters was successful
   */
  bool loadParams(const vigir_generic_params::ParameterSet& /*global_params*/) override;

  /**
   * @brief Resets the plugin to initial state.
   */
  void reset() override;

  /**
   * @brief Generates list of valid adjacent predecessor states. The returned states will be pushed back to the ARA* open list.
   * Therefore all states must be completely derived and cross-checked including:
   * - cost and risk (StepCostEstimator)
   * - 3D pose (WorldModel)
   * - post processing (PostProcessor)
   * - collision checks (WorldModel)
   * - reachability (RobotModel)
   * @param state Current state from which all valid adjacent states should be determined.
   * @return List of all valid adjacent states
   */
  std::list<PlanningState::Ptr> generatePredecessor(const PlanningState& state) const override;

  /**
   * @brief Generates list of valid adjacent successor states. The returned states will be pushed back to the ARA* open list.
   * Therefore all states must be completely defined and checked including:
   * - cost and risk (StepCostEstimator)
   * - 3D pose (WorldModel)
   * - post processing (PostProcessor)
   * - collision checks (WorldModel)
   * - reachability (RobotModel)
   * @param state Current state from which all valid adjacent states should be determined.
   * @return List of all valid adjacent states
   */
  std::list<PlanningState::Ptr> generateSuccessor(const PlanningState& state) const override;
};
}

#endif
