#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "arx_lift_src/lift_head_control_loop.h"

namespace py = pybind11;

PYBIND11_MODULE(arx_lift_python, m) {
    py::enum_<arx::LiftHeadControlLoop::RobotType>(m,"RobotType")
        .value("LIFT",arx::LiftHeadControlLoop::RobotType::LIFT)
        .value("X7S",arx::LiftHeadControlLoop::RobotType::X7S)
        .export_values();

    py::class_<arx::LiftHeadControlLoop>(m, "LiftHeadControlLoop")
        .def(py::init<const char *,arx::LiftHeadControlLoop::RobotType>())
        .def("set_height", &arx::LiftHeadControlLoop::setHeight)
        .def("set_waist_pos", &arx::LiftHeadControlLoop::setWaistPos)
        .def("set_head_yaw", &arx::LiftHeadControlLoop::setHeadYaw)
        .def("set_head_pitch", &arx::LiftHeadControlLoop::setHeadPitch)
        .def("set_chassis_cmd", &arx::LiftHeadControlLoop::setChassisCmd)
        .def("get_height",&arx::LiftHeadControlLoop::getHeight)
        .def("get_waist_pos",&arx::LiftHeadControlLoop::getWaistPos)
        .def("get_head_yaw",&arx::LiftHeadControlLoop::getHeadYaw)
        .def("get_head_pitch",&arx::LiftHeadControlLoop::getHeadPitch)
        .def("read", &arx::LiftHeadControlLoop::read)
        .def("update", &arx::LiftHeadControlLoop::update)
        .def("write", &arx::LiftHeadControlLoop::write)
        .def("loop", &arx::LiftHeadControlLoop::loop);
}
