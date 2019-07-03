// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#ifndef MODULES_WORLD_EVALUATION_SPEED_LIMIT_HPP_
#define MODULES_WORLD_EVALUATION_SPEED_LIMIT_HPP_

#include <memory>

#include "modules/world/evaluation/base_evaluator.hpp"
#include "modules/world/objects/agent.hpp"

namespace modules
{
namespace world
{

class World;
namespace evaluation
{

class EvaluatorSpeedLimit : public BaseEvaluator
{
  public:
    EvaluatorSpeedLimit(const AgentId& agent_id, double speed_limit) : agent_id_(agent_id), speed_limit_(speed_limit)  {};
    virtual ~EvaluatorSpeedLimit() {};

    virtual EvaluationReturn Evaluate(const world::World& world) const { return world.get_agents()[agent_id_]->get_current_state()[4] < speed_limit_;}

  private: 
    AgentId agent_id_;
    double speed_limit_;
};

} // namespace collision
} // namespace world
} // namespace modules

#endif // MODULES_WORLD_EVALUATION_EVALUATOR_SPEED_LIMIT_HPP_