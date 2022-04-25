#!/usr/bin/python2
# -*- coding: utf-8 -*-

# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""This file generates shell code for the setup.SHELL scripts to set environment variables."""

from __future__ import print_function

import argparse
import copy
import errno
import os
import platform
import sys

CATKIN_MARKER_FILE = '.catkin'

system = platform.system()
IS_DARWIN = (system == 'Darwin')
IS_WINDOWS = (system == 'Windows')

PATH_TO_ADD_SUFFIX = ['bin']
if IS_WINDOWS:
    # while catkin recommends putting dll's into bin, 3rd party packages often put dll's into lib
    # since Windows finds dll's via the PATH variable, prepend it with path to lib
    PATH_TO_ADD_SUFFIX.extend([['lib', os.path.join('lib', 'x86_64-linux-gnu')]])

# subfolder of workspace prepended to CMAKE_PREFIX_PATH
ENV_VAR_SUBFOLDERS = {
    'CMAKE_PREFIX_PATH': '',
    'LD_LIBRARY_PATH' if not IS_DARWIN else 'DYLD_LIBRARY_PATH': ['lib', os.path.join('lib', 'x86_64-linux-gnu')],
    'PATH': PATH_TO_ADD_SUFFIX,
    'PKG_CONFIG_PATH': [os.path.join('lib', 'pkgconfig'), os.path.join('lib', 'x86_64-linux-gnu', 'pkgconfig')],
    'PYTHONPATH': 'lib/python2.7/dist-packages',
}


def rollback_env_variables(environ, env_var_subfolders):
    """
    Generate shell code to reset environment variables.

    by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH.
    This does not cover modifications performed by environment hooks.
    """
    lines = []
    unmodified_environ = copy.copy(environ)
    for key in sorted(env_var_subfolders.keys()):
        subfolders = env_var_subfolders[key]
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        value = _rollback_env_variable(unmodified_environ, key, subfolders)
        if value is not None:
            environ[key] = value
            lines.append(assignment(key, value))
    if lines:
        lines.insert(0, comment('reset environment variables by unrolling modifications based on all workspaces in CMAKE_PREFIX_PATH'))
    return lines


def _rollback_env_variable(environ, name, subfolders):
    """
    For each catkin workspace in CMAKE_PREFIX_PATH remove the first entry from env[NAME] matching workspace + subfolder.

    :param subfolders: list of str '' or subfoldername that may start with '/'
    :returns: the updated value of the environment variable.
    """
    value = environ[name] if name in environ else ''
    env_paths = [path for path in value.split(os.pathsep) if path]
    value_modified = False
    for subfolder in subfolders:
        if subfolder:
            if subfolder.startswith(os.path.sep) or (os.path.altsep and subfolder.startswith(os.path.altsep)):
                subfolder = subfolder[1:]
            if subfolder.endswith(os.path.sep) or (os.path.altsep and subfolder.endswith(os.path.altsep)):
                subfolder = subfolder[:-1]
        for ws_path in _get_workspaces(environ, include_fuerte=True, include_non_existing=True):
            path_to_find = os.path.join(ws_path, subfolder) if subfolder else ws_path
            path_to_remove = None
            for env_path in env_paths:
                env_path_clean = env_path[:-1] if env_path and env_path[-1] in [os.path.sep, os.path.altsep] else env_path
                if env_path_clean == path_to_find:
                    path_to_remove = env_path
                    break
            if path_to_remove:
                env_paths.remove(path_to_remove)
                value_modified = True
    new_value = os.pathsep.join(env_paths)
    return new_value if value_modified else None


def _get_workspaces(environ, include_fuerte=False, include_non_existing=False):
    """
    Based on CMAKE_PREFIX_PATH return all catkin workspaces.

    :param include_fuerte: The flag if paths starting with '/opt/ros/fuerte' should be considered workspaces, ``bool``
    """
    # get all cmake prefix paths
    env_name = 'CMAKE_PREFIX_PATH'
    value = environ[env_name] if env_name in environ else ''
    paths = [path for path in value.split(os.pathsep) if path]
    # remove non-workspace paths
    workspaces = [path for path in paths if os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE)) or (include_fuerte and path.startswith('/opt/ros/fuerte')) or (include_non_existing and not os.path.exists(path))]
    return workspaces


def prepend_env_variables(environ, env_var_subfolders, workspaces):
    """Generate shell code to prepend environment variables for the all workspaces."""
    lines = []
    lines.append(comment('prepend folders of workspaces to environment variables'))

    paths = [path for path in workspaces.split(os.pathsep) if path]

    prefix = _prefix_env_variable(environ, 'CMAKE_PREFIX_PATH', paths, '')
    lines.append(prepend(environ, 'CMAKE_PREFIX_PATH', prefix))

    for key in sorted(key for key in env_var_subfolders.keys() if key != 'CMAKE_PREFIX_PATH'):
        subfolder = env_var_subfolders[key]
        prefix = _prefix_env_variable(environ, key, paths, subfolder)
        lines.append(prepend(environ, key, prefix))
    return lines


def _prefix_env_variable(environ, name, paths, subfolders):
    """
    Return the prefix to prepend to the environment variable NAME.

    Adding any path in NEW_PATHS_STR without creating duplicate or empty items.
    """
    value = environ[name] if name in environ else ''
    environ_paths = [path for path in value.split(os.pathsep) if path]
    checked_paths = []
    for path in paths:
        if not isinstance(subfolders, list):
            subfolders = [subfolders]
        for subfolder in subfolders:
            path_tmp = path
            if subfolder:
                path_tmp = os.path.join(path_tmp, subfolder)
            # skip nonexistent paths
            if not os.path.exists(path_tmp):
                continue
            # exclude any path already in env and any path we already added
            if path_tmp not in environ_paths and path_tmp not in checked_paths:
                checked_paths.append(path_tmp)
    prefix_str = os.pathsep.join(checked_paths)
    if prefix_str != '' and environ_paths:
        prefix_str += os.pathsep
    return prefix_str


def assignment(key, value):
    if not IS_WINDOWS:
        return 'export %s="%s"' % (key, value)
    else:
        return 'set %s=%s' % (key, value)


def comment(msg):
    if not IS_WINDOWS:
        return '# %s' % msg
    else:
        return 'REM %s' % msg


def prepend(environ, key, prefix):
    if key not in environ or not environ[key]:
        return assignment(key, prefix)
    if not IS_WINDOWS:
        return 'export %s="%s$%s"' % (key, prefix, key)
    else:
        return 'set %s=%s%%%s%%' % (key, prefix, key)


def find_env_hooks(environ, cmake_prefix_path):
    """Generate shell code with found environment hooks for the all workspaces."""
    lines = []
    lines.append(comment('found environment hooks in workspaces'))

    generic_env_hooks = []
    generic_env_hooks_workspace = []
    specific_env_hooks = []
    specific_env_hooks_workspace = []
    generic_env_hooks_by_filename = {}
    specific_env_hooks_by_filename = {}
    generic_env_hook_ext = 'bat' if IS_WINDOWS else 'sh'
    specific_env_hook_ext = environ['CATKIN_SHELL'] if not IS_WINDOWS and 'CATKIN_SHELL' in environ and environ['CATKIN_SHELL'] else None
    # remove non-workspace paths
    workspaces = [path for path in cmake_prefix_path.split(os.pathsep) if path and os.path.isfile(os.path.join(path, CATKIN_MARKER_FILE))]
    for workspace in reversed(workspaces):
        env_hook_dir = os.path.join(workspace, 'etc', 'catkin', 'profile.d')
        if os.path.isdir(env_hook_dir):
            for filename in sorted(os.listdir(env_hook_dir)):
                if filename.endswith('.%s' % generic_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in generic_env_hooks_by_filename:
                        i = generic_env_hooks.index(generic_env_hooks_by_filename[filename])
                        generic_env_hooks.pop(i)
                        generic_env_hooks_workspace.pop(i)
                    # append env hook
                    generic_env_hooks.append(os.path.join(env_hook_dir, filename))
                    generic_env_hooks_workspace.append(workspace)
                    generic_env_hooks_by_filename[filename] = generic_env_hooks[-1]
                elif specific_env_hook_ext is not None and filename.endswith('.%s' % specific_env_hook_ext):
                    # remove previous env hook with same name if present
                    if filename in specific_env_hooks_by_filename:
                        i = specific_env_hooks.index(specific_env_hooks_by_filename[filename])
                        specific_env_hooks.pop(i)
                        specific_env_hooks_workspace.pop(i)
                    # append env hook
                    specific_env_hooks.append(os.path.join(env_hook_dir, filename))
                    specific_env_hooks_workspace.append(workspace)
                    specific_env_hooks_by_filename[filename] = specific_env_hooks[-1]
    env_hooks = generic_env_hooks + specific_env_hooks
    env_hooks_workspace = generic_env_hooks_workspace + specific_env_hooks_workspace
    count = len(env_hooks)
    lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_COUNT', count))
    for i in range(count):
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d' % i, env_hooks[i]))
        lines.append(assignment('_CATKIN_ENVIRONMENT_HOOKS_%d_WORKSPACE' % i, env_hooks_workspace[i]))
    return lines


def _parse_arguments(args=None):
    parser = argparse.ArgumentParser(description='Generates code blocks for the setup.SHELL script.')
    parser.add_argument('--extend', action='store_true', help='Skip unsetting previous environment variables to extend context')
    parser.add_argument('--local', action='store_true', help='Only consider this prefix path and ignore other prefix path in the environment')
    return parser.parse_known_args(args=args)[0]


if __name__ == '__main__':
    try:
        try:
            args = _parse_arguments()
        except Exception as e:
            print(e, file=sys.stderr)
            sys.exit(1)

        if not args.local:
            # environment at generation time
            CMAKE_PREFIX_PATH = r'/home/beast/rviz_alarms_ws/devel;/opt/ros/melodic;/home/beast/ros2_dashing/install/rosbag2_transport;/home/beast/ros2_dashing/install/robot_state_publisher;/home/beast/ros2_dashing/install/tf2_sensor_msgs;/home/beast/ros2_dashing/install/tf2_kdl;/home/beast/ros2_dashing/install/tf2_geometry_msgs;/home/beast/ros2_dashing/install/tf2_eigen;/home/beast/ros2_dashing/install/tf2_ros;/home/beast/ros2_dashing/install/rosbag2_storage_default_plugins;/home/beast/ros2_dashing/install/rosbag2_converter_default_plugins;/home/beast/ros2_dashing/install/rosbag2;/home/beast/ros2_dashing/install/rosbag2_storage;/home/beast/ros2_dashing/install/yaml_cpp_vendor;/home/beast/ros2_dashing/install/common_interfaces;/home/beast/ros2_dashing/install/visualization_msgs;/home/beast/ros2_dashing/install/kdl_parser;/home/beast/ros2_dashing/install/urdf;/home/beast/ros2_dashing/install/urdfdom;/home/beast/ros2_dashing/install/turtlesim;/home/beast/ros2_dashing/install/tf2_msgs;/home/beast/ros2_dashing/install/test_msgs;/home/beast/ros2_dashing/install/sros2_cmake;/home/beast/ros2_dashing/install/rqt_publisher;/home/beast/ros2_dashing/install/rqt_plot;/home/beast/ros2_dashing/install/rqt_msg;/home/beast/ros2_dashing/install/rqt_console;/home/beast/ros2_dashing/install/rqt_py_common;/home/beast/ros2_dashing/install/ros_testing;/home/beast/ros2_dashing/install/lifecycle;/home/beast/ros2_dashing/install/quality_of_service_demo_cpp;/home/beast/ros2_dashing/install/demo_nodes_cpp;/home/beast/ros2_dashing/install/composition;/home/beast/ros2_dashing/install/action_tutorials;/home/beast/ros2_dashing/install/rclpy;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_action_server;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_action_client;/home/beast/ros2_dashing/install/rclcpp_action;/home/beast/ros2_dashing/install/rcl_action;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_service;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_client;/home/beast/ros2_dashing/install/example_interfaces;/home/beast/ros2_dashing/install/action_msgs;/home/beast/ros2_dashing/install/unique_identifier_msgs;/home/beast/ros2_dashing/install/ament_lint_common;/home/beast/ros2_dashing/install/ament_cmake_uncrustify;/home/beast/ros2_dashing/install/uncrustify_vendor;/home/beast/ros2_dashing/install/trajectory_msgs;/home/beast/ros2_dashing/install/pendulum_control;/home/beast/ros2_dashing/install/tlsf_cpp;/home/beast/ros2_dashing/install/tlsf;/home/beast/ros2_dashing/install/rqt_gui_cpp;/home/beast/ros2_dashing/install/qt_gui_core;/home/beast/ros2_dashing/install/qt_gui_cpp;/home/beast/ros2_dashing/install/pluginlib;/home/beast/ros2_dashing/install/rosbag2_test_common;/home/beast/ros2_dashing/install/rclcpp_lifecycle;/home/beast/ros2_dashing/install/logging_demo;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_composition;/home/beast/ros2_dashing/install/rclcpp_components;/home/beast/ros2_dashing/install/laser_geometry;/home/beast/ros2_dashing/install/intra_process_demo;/home/beast/ros2_dashing/install/image_tools;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_timer;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_subscriber;/home/beast/ros2_dashing/install/examples_rclcpp_minimal_publisher;/home/beast/ros2_dashing/install/dummy_sensors;/home/beast/ros2_dashing/install/dummy_map_server;/home/beast/ros2_dashing/install/demo_nodes_cpp_native;/home/beast/ros2_dashing/install/rclcpp;/home/beast/ros2_dashing/install/rcl_lifecycle;/home/beast/ros2_dashing/install/rcl;/home/beast/ros2_dashing/install/tf2;/home/beast/ros2_dashing/install/test_security;/home/beast/ros2_dashing/install/test_quality_of_service;/home/beast/ros2_dashing/install/test_launch_testing;/home/beast/ros2_dashing/install/test_cli_remapping;/home/beast/ros2_dashing/install/test_cli;/home/beast/ros2_dashing/install/stereo_msgs;/home/beast/ros2_dashing/install/std_srvs;/home/beast/ros2_dashing/install/shape_msgs;/home/beast/ros2_dashing/install/map_msgs;/home/beast/ros2_dashing/install/sensor_msgs;/home/beast/ros2_dashing/install/nav_msgs;/home/beast/ros2_dashing/install/diagnostic_msgs;/home/beast/ros2_dashing/install/geometry_msgs;/home/beast/ros2_dashing/install/actionlib_msgs;/home/beast/ros2_dashing/install/std_msgs;/home/beast/ros2_dashing/install/shared_queues_vendor;/home/beast/ros2_dashing/install/rviz_rendering_tests;/home/beast/ros2_dashing/install/rviz_rendering;/home/beast/ros2_dashing/install/rviz_ogre_vendor;/home/beast/ros2_dashing/install/rviz_assimp_vendor;/home/beast/ros2_dashing/install/rttest;/home/beast/ros2_dashing/install/rosgraph_msgs;/home/beast/ros2_dashing/install/composition_interfaces;/home/beast/ros2_dashing/install/rcl_interfaces;/home/beast/ros2_dashing/install/pendulum_msgs;/home/beast/ros2_dashing/install/lifecycle_msgs;/home/beast/ros2_dashing/install/builtin_interfaces;/home/beast/ros2_dashing/install/rosidl_default_runtime;/home/beast/ros2_dashing/install/rosidl_default_generators;/home/beast/ros2_dashing/install/rosidl_typesupport_cpp;/home/beast/ros2_dashing/install/rosidl_generator_py;/home/beast/ros2_dashing/install/rosidl_typesupport_c;/home/beast/ros2_dashing/install/rmw_implementation;/home/beast/ros2_dashing/install/rmw_opensplice_cpp;/home/beast/ros2_dashing/install/rosidl_typesupport_opensplice_c;/home/beast/ros2_dashing/install/rosidl_typesupport_opensplice_cpp;/home/beast/ros2_dashing/install/rmw_fastrtps_dynamic_cpp;/home/beast/ros2_dashing/install/rmw_cyclonedds_cpp;/home/beast/ros2_dashing/install/rosidl_typesupport_introspection_cpp;/home/beast/ros2_dashing/install/rosidl_typesupport_introspection_c;/home/beast/ros2_dashing/install/rmw_fastrtps_cpp;/home/beast/ros2_dashing/install/rosidl_typesupport_fastrtps_c;/home/beast/ros2_dashing/install/rosidl_typesupport_fastrtps_cpp;/home/beast/ros2_dashing/install/rmw_connext_cpp;/home/beast/ros2_dashing/install/rosidl_typesupport_connext_c;/home/beast/ros2_dashing/install/rosidl_typesupport_connext_cpp;/home/beast/ros2_dashing/install/rosidl_generator_cpp;/home/beast/ros2_dashing/install/rmw_fastrtps_shared_cpp;/home/beast/ros2_dashing/install/rmw;/home/beast/ros2_dashing/install/rosidl_generator_c;/home/beast/ros2_dashing/install/rosidl_typesupport_interface;/home/beast/ros2_dashing/install/rosidl_generator_dds_idl;/home/beast/ros2_dashing/install/rosidl_cmake;/home/beast/ros2_dashing/install/rosidl_parser;/home/beast/ros2_dashing/install/rosidl_adapter;/home/beast/ros2_dashing/install/rmw_implementation_cmake;/home/beast/ros2_dashing/install/rmw_connext_shared_cpp;/home/beast/ros2_dashing/install/resource_retriever;/home/beast/ros2_dashing/install/rcl_logging_noop;/home/beast/ros2_dashing/install/rcl_logging_log4cxx;/home/beast/ros2_dashing/install/rcutils;/home/beast/ros2_dashing/install/rcpputils;/home/beast/ros2_dashing/install/rcl_yaml_param_parser;/home/beast/ros2_dashing/install/qt_gui_py_common;/home/beast/ros2_dashing/install/qt_gui_app;/home/beast/ros2_dashing/install/qt_gui;/home/beast/ros2_dashing/install/qt_dotgraph;/home/beast/ros2_dashing/install/python_qt_binding;/home/beast/ros2_dashing/install/launch_testing_ament_cmake;/home/beast/ros2_dashing/install/python_cmake_module;/home/beast/ros2_dashing/install/class_loader;/home/beast/ros2_dashing/install/fastrtps;/home/beast/ros2_dashing/install/orocos_kdl;/home/beast/ros2_dashing/install/opensplice_cmake_module;/home/beast/ros2_dashing/install/message_filters;/home/beast/ros2_dashing/install/libyaml_vendor;/home/beast/ros2_dashing/install/libcurl_vendor;/home/beast/ros2_dashing/install/ament_cmake_gmock;/home/beast/ros2_dashing/install/ament_cmake_gtest;/home/beast/ros2_dashing/install/fastrtps_cmake_module;/home/beast/ros2_dashing/install/eigen3_cmake_module;/home/beast/ros2_dashing/install/cyclonedds_cmake_module;/home/beast/ros2_dashing/install/console_bridge_vendor;/home/beast/ros2_dashing/install/connext_cmake_module;/home/beast/ros2_dashing/install/ament_cmake_xmllint;/home/beast/ros2_dashing/install/ament_cmake_pyflakes;/home/beast/ros2_dashing/install/ament_cmake_pep8;/home/beast/ros2_dashing/install/ament_cmake_pep257;/home/beast/ros2_dashing/install/ament_cmake_pclint;/home/beast/ros2_dashing/install/ament_lint_auto;/home/beast/ros2_dashing/install/ament_cmake_ros;/home/beast/ros2_dashing/install/ament_cmake_pytest;/home/beast/ros2_dashing/install/ament_cmake_nose;/home/beast/ros2_dashing/install/ament_cmake_lint_cmake;/home/beast/ros2_dashing/install/ament_cmake_flake8;/home/beast/ros2_dashing/install/ament_cmake_cpplint;/home/beast/ros2_dashing/install/ament_cmake_cppcheck;/home/beast/ros2_dashing/install/ament_cmake_copyright;/home/beast/ros2_dashing/install/ament_cmake_clang_tidy;/home/beast/ros2_dashing/install/ament_cmake_clang_format;/home/beast/ros2_dashing/install/ament_cmake_auto;/home/beast/ros2_dashing/install/ament_cmake;/home/beast/ros2_dashing/install/ament_cmake_target_dependencies;/home/beast/ros2_dashing/install/ament_index_cpp;/home/beast/ros2_dashing/install/urdfdom_headers;/home/beast/ros2_dashing/install/tinyxml_vendor;/home/beast/ros2_dashing/install/tinyxml2_vendor;/home/beast/ros2_dashing/install/tinydir_vendor;/home/beast/ros2_dashing/install/test_interface_files;/home/beast/ros2_dashing/install/sqlite3_vendor;/home/beast/ros2_dashing/install/ros_environment;/home/beast/ros2_dashing/install/poco_vendor;/home/beast/ros2_dashing/install/osrf_testing_tools_cpp;/home/beast/ros2_dashing/install/gmock_vendor;/home/beast/ros2_dashing/install/gtest_vendor;/home/beast/ros2_dashing/install/fastcdr;/home/beast/ros2_dashing/install/cyclonedds;/home/beast/ros2_dashing/install/ament_cmake_test;/home/beast/ros2_dashing/install/ament_cmake_python;/home/beast/ros2_dashing/install/ament_cmake_export_dependencies;/home/beast/ros2_dashing/install/ament_cmake_libraries;/home/beast/ros2_dashing/install/ament_cmake_include_directories;/home/beast/ros2_dashing/install/ament_cmake_export_link_flags;/home/beast/ros2_dashing/install/ament_cmake_export_interfaces;/home/beast/ros2_dashing/install/ament_cmake_export_libraries;/home/beast/ros2_dashing/install/ament_cmake_export_include_directories;/home/beast/ros2_dashing/install/ament_cmake_export_definitions;/home/beast/ros2_dashing/install/ament_cmake_core'.split(';')
        else:
            # don't consider any other prefix path than this one
            CMAKE_PREFIX_PATH = []
        # prepend current workspace if not already part of CPP
        base_path = os.path.dirname(__file__)
        # CMAKE_PREFIX_PATH uses forward slash on all platforms, but __file__ is platform dependent
        # base_path on Windows contains backward slashes, need to be converted to forward slashes before comparison
        if os.path.sep != '/':
            base_path = base_path.replace(os.path.sep, '/')

        if base_path not in CMAKE_PREFIX_PATH:
            CMAKE_PREFIX_PATH.insert(0, base_path)
        CMAKE_PREFIX_PATH = os.pathsep.join(CMAKE_PREFIX_PATH)

        environ = dict(os.environ)
        lines = []
        if not args.extend:
            lines += rollback_env_variables(environ, ENV_VAR_SUBFOLDERS)
        lines += prepend_env_variables(environ, ENV_VAR_SUBFOLDERS, CMAKE_PREFIX_PATH)
        lines += find_env_hooks(environ, CMAKE_PREFIX_PATH)
        print('\n'.join(lines))

        # need to explicitly flush the output
        sys.stdout.flush()
    except IOError as e:
        # and catch potential "broken pipe" if stdout is not writable
        # which can happen when piping the output to a file but the disk is full
        if e.errno == errno.EPIPE:
            print(e, file=sys.stderr)
            sys.exit(2)
        raise

    sys.exit(0)
