// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include "behavior.hpp"
#include "modules/models/behavior/constant_velocity/constant_velocity.hpp"
#include "modules/models/behavior/rrt_star/rrt_star.hpp"
#include "modules/models/behavior/motion_primitives/motion_primitives.hpp"

namespace py = pybind11;
using modules::models::behavior::BehaviorModel;
using modules::models::behavior::BehaviorModelPtr;
using modules::models::behavior::BehaviorConstantVelocity;
using modules::models::behavior::BehaviorRRTStar;
using modules::models::behavior::BehaviorMotionPrimitives;

using std::shared_ptr;
void python_behavior(py::module m) {
  py::class_<BehaviorModel,
             PyBehaviorModel,
             BehaviorModelPtr>(m, "BehaviorModel")
      .def(py::init<modules::commons::Params *>())
      .def("plan", &BehaviorModel::Plan)
      .def("clone", &BehaviorModel::Clone)
      .def_property("last_trajectory",
                    &BehaviorModel::get_last_trajectory,
                    &BehaviorModel::set_last_trajectory);

  py::class_<BehaviorConstantVelocity,
             BehaviorModel,
             shared_ptr<BehaviorConstantVelocity>>(m,
                                                   "BehaviorConstantVelocity")
      .def(py::init<modules::commons::Params *>())
      .def("__repr__", [](const BehaviorConstantVelocity &m) {
        return "bark.behavior.BehaviorConstantVelocity";
      })
      .def(py::pickle(
        [](const BehaviorConstantVelocity &b) { 
            return py::make_tuple(b.get_last_trajectory()); // 0
        },
        [](py::tuple t) { // __setstate__
            if (t.size() != 1)
                throw std::runtime_error("Invalid behavior model state!");

            /* Create a new C++ instance */
            return new BehaviorConstantVelocity(nullptr); // param pointer must be set afterwards
        }));

    py::class_<BehaviorRRTStar,
             BehaviorModel,
             shared_ptr<BehaviorRRTStar>>(m,
                                                   "BehaviorRRTStar")
      .def(py::init<modules::commons::Params *>())
      .def("__repr__", [](const BehaviorRRTStar &m) {
        return "bark.behavior.BehaviorRRTStar";
      })
      .def(py::pickle(
        [](const BehaviorRRTStar &b) { 
            return py::make_tuple(b.get_last_trajectory()); // 0
        },
        [](py::tuple t) { // __setstate__
            if (t.size() != 1)
                throw std::runtime_error("Invalid behavior model state!");

            /* Create a new C++ instance */
            return new BehaviorRRTStar(nullptr); // param pointer must be set afterwards
        }));

  py::class_<BehaviorMotionPrimitives,
             BehaviorModel,
             shared_ptr<BehaviorMotionPrimitives>>(m, "BehaviorMotionPrimitives")
      .def(py::init<const modules::models::dynamic::DynamicModelPtr&, modules::commons::Params *>())
      .def("__repr__", [](const BehaviorMotionPrimitives &b) {
        return "bark.behavior.BehaviorMotionPrimitives";
      })
      .def("add_motion_primitive", &BehaviorMotionPrimitives::AddMotionPrimitive)
      .def("action_to_behavior", &BehaviorMotionPrimitives::ActionToBehavior);
}