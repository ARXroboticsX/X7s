#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "arx_x7_src/interfaces/InterfacesPy.hpp"

namespace py = pybind11;

PYBIND11_MODULE(arx_x7_python, m) {
    py::class_<arx::x7::InterfacesPy>(m, "InterfacesPy")
        .def(py::init<std::string,std::string,int>()) 
        .def("set_joint_positions", &arx::x7::InterfacesPy::set_joint_positions)
        .def("set_ee_pose", &arx::x7::InterfacesPy::set_ee_pose)
        .def("set_arm_status", &arx::x7::InterfacesPy::set_arm_status)
        .def("set_catch", &arx::x7::InterfacesPy::set_catch)
        .def("get_joint_positions", &arx::x7::InterfacesPy::get_joint_positions)
        .def("get_joint_velocities", &arx::x7::InterfacesPy::get_joint_velocities)
        .def("get_joint_currents", &arx::x7::InterfacesPy::get_joint_currents)
        .def("arx_x", &arx::x7::InterfacesPy::arx_x)
        .def("get_ee_pose", &arx::x7::InterfacesPy::get_ee_pose);
}
