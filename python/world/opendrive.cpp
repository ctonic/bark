// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.


#include <string>
#include "opendrive.hpp"
#include "modules/world/opendrive/commons.hpp"
#include "modules/world/opendrive/plan_view.hpp"
#include "modules/world/opendrive/road.hpp"
#include "modules/world/opendrive/lane_section.hpp"
#include "modules/world/opendrive/opendrive.hpp"
#include "modules/world/opendrive/odrSpiral.hpp"

using namespace modules::world::opendrive;

void python_opendrive(py::module m) {
  py::class_<PlanView, std::shared_ptr<PlanView>>(m, "PlanView")
      .def(py::init<>())
      .def("add_line", &PlanView::add_line, "Add line to planview")
      .def("add_spiral", &PlanView::add_spiral, "Add spiral to planview")
      .def("add_arc", &PlanView::add_arc, "Add arc to planview")
      .def("get_reference_line", &PlanView::get_reference_line, "Return as numpy array")
      .def("create_lane", &PlanView::create_lane, "Creates lane with offset")
      .def("add_paramPoly3", &PlanView::add_paramPoly3, "Add parametric cubic curve to planview");

  py::class_<LinkInfo>(m, "LinkInfo")
      .def(py::init<>())
      .def_readwrite("id", &LinkInfo::position_)
      .def_readwrite("type", &LinkInfo::type_);

  py::class_<Link>(m, "Link")
      .def(py::init<>())
      .def_property("predecessor", &Link::get_predecessor, &Link::set_predecessor)
      .def_property("successor", &Link::get_successor, &Link::set_successor)
      .def(
          "__repr__",
          [](const Link &l) {
          std::stringstream ss;
          ss << "<bark.Link> Link: ";
          ss << modules::world::opendrive::print(l);
          return ss.str();
      });

  py::class_<RoadMark>(m, "RoadMark")
      .def(py::init<>())
      .def_readwrite("type", &RoadMark::type_)
      .def_readwrite("color", &RoadMark::color_)
      .def_readwrite("width", &RoadMark::width_);

  py::class_<Lane, std::shared_ptr<Lane>>(m, "Lane")
      .def(py::init<>())
      .def(py::init<LanePosition&>())
      .def_property("lane_id", &Lane::get_id, &Lane::set_id)
      .def_property("lane_position", &Lane::get_lane_position, &Lane::set_lane_position)
      .def_property("link", &Lane::get_link, &Lane::set_link)
      .def_property("line", &Lane::get_line, &Lane::set_line)
      .def_property("road_mark", &Lane::get_road_mark, &Lane::set_road_mark)
      .def_property("speed", &Lane::get_speed, &Lane::set_speed)
      .def(
          "__repr__",
          [](const Lane &l) {
          std::stringstream ss;
          ss << "<bark.Lane> Lane: ";
          ss << modules::world::opendrive::print(l);
          return ss.str();
      });

  py::class_<LaneOffset>(m, "LaneOffset")
      .def(py::init<float, float, float, float>());

  py::class_<LaneWidth>(m, "LaneWidth")
      .def(py::init<float, float, LaneOffset>());

  py::class_<LaneWidths>(m, "LaneWidths")
      .def(py::init<>());

  py::class_<LaneSection, std::shared_ptr<LaneSection>>(m, "LaneSection")
      .def(py::init<float>())
      .def("add_lane", &LaneSection::add_lane, "Add lane element")
      .def("get_lanes", &LaneSection::get_lanes, "Get all lane elements")
      .def("get_left_lane", &LaneSection::get_left_lane, "Get left lane")
      .def("get_right_lane", &LaneSection::get_right_lane, "Get right lane");

  py::class_<Road, std::shared_ptr<Road>>(m, "Road")
      .def(py::init<>())
      .def_property("name", &Road::get_name, &Road::set_name)
      .def_property("id", &Road::get_id, &Road::set_id)
      .def_property("plan_view", &Road::get_plan_view, &Road::set_plan_view)
      .def_property("link", &Road::get_link, &Road::set_link)
      .def_property_readonly("lane_sections", &Road::get_lane_sections)
      .def("add_lane_section", &Road::add_lane_section, "Add a lane section to road");

  py::class_<OpenDriveMap, std::shared_ptr<OpenDriveMap>>(m, "OpenDriveMap")
      .def(py::init<>())
      .def("add_road", &OpenDriveMap::add_road, "Add road element")
      .def("add_junction", &OpenDriveMap::add_junction, "Add junction element")
      .def("get_road", &OpenDriveMap::get_road, "Get road element")
      .def("get_roads", &OpenDriveMap::get_roads, "Get all roads")
      .def("get_junctions", &OpenDriveMap::get_junctions, "Get all junctions");

  py::class_<LaneLink>(m, "LaneLink")
      .def(py::init<>())
      .def_readwrite("from_id", &LaneLink::from_id)
      .def_readwrite("to_id", &LaneLink::to_id);

  py::class_<Connection>(m, "Connection")
      .def(py::init<>())
      .def_readwrite("id", &Connection::id_)
      .def_readwrite("incoming_road", &Connection::incoming_road_)
      .def_readwrite("connecting_road", &Connection::connecting_road_)
      .def_property_readonly("lane_links", &Connection::get_lane_links)
      .def("add_lane_link", &Connection::add_lane_link, "Add add LaneLink");

  py::class_<Junction, std::shared_ptr<Junction>>(m, "Junction")
      .def_property("id", &Junction::get_id, &Junction::set_id)
      .def("get_connections", &Junction::get_connections)
      .def("add_connection", &Junction::add_connection)
      .def(py::init<std::string, int>());

  m.def("fresnel_cos", &fresnelCos);
  m.def("fresnel_sin", &fresnelSin);

}
