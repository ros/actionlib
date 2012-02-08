#!/usr/bin/env python

from setuptools import setup

import os
import sys
sys.path.insert(0, 'src')

version = '0.0.0'

PKG = 'actionlib'
gen = ['msg']
packages = [PKG]
package_dir = {PKG: 'src/%s'%PKG}
if 'CATKIN_BINARY_DIR' in os.environ:
     build_d = os.environ['CATKIN_BINARY_DIR']
     for t in gen:
         p = os.path.join(build_d, 'gen', 'py', PKG, t)
         if os.path.isdir(p):
             # e.g. std_msgs.msg = build/gen/py/std_msgs/msg
             package_dir["%s.%s"%(PKG, t)] = p
             packages.append("%s.%s"%(PKG, t))

setup(name=PKG,
      version=version,
      packages=packages,
      package_dir = package_dir,
      install_requires=[],
      scripts = [],
      author = "Eitan Marder-Eppstein, Vijay Pradeep", 
      author_email = "eitan@willowgarage.com",
      url = "http://www.ros.org/wiki/%s"%(PKG),
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "ROS library for pre-emptable tasks",
      long_description = "The actionlib package provides a standardized interface for interfacing with preemptible tasks. Examples of this include moving the base to a target location, performing a laser scan and returning the resulting point cloud, detecting the handle of a door, etc.",
      license = "BSD"
      )
