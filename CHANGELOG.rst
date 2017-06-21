^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package actionlib
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.11.9 (2017-03-27)
-------------------
* Python3 compatibility + pep8 compliance (`#71 <https://github.com/ros/actionlib/issues/71>`_) follow-up of (`#43 <https://github.com/ros/actionlib/issues/43>`_)
* - wait for ros::Time::now to become valid before init of connection_monitor (`#62 <https://github.com/ros/actionlib/issues/62>`_)
  - bugfix : connection_monitor should wait for result
* fixed default value for rosparam. closes `#69 <https://github.com/ros/actionlib/issues/69>`_ (`#70 <https://github.com/ros/actionlib/issues/70>`_)
* Contributors: 1r0b1n0, Mikael Arguedas, Piyush Khandelwal

1.11.8 (2017-02-17)
-------------------
* Fixes a deadlock (`#64 <https://github.com/ros/actionlib/issues/64>`_)
* Removed unused variables warnings (`#63 <https://github.com/ros/actionlib/issues/63>`_ `#65 <https://github.com/ros/actionlib/issues/65>`_)
* If using sim time, wait for /clock (`#59 <https://github.com/ros/actionlib/issues/59>`_)
* add parameters to configure queue sizes (`#55 <https://github.com/ros/actionlib/pull/55>`_)
* Contributors: Esteve Fernandez, Jonathan Meyer, Mikael Arguedas, Patrick Beeson, Robin Vanhove

1.11.7 (2016-10-24)
-------------------
* Merge pull request `#57 <https://github.com/ros/actionlib/issues/57>`_ from stonier/patch-1
  Remove misleading error log
* Remove misleading error log
  This was introduced in https://github.com/ros/actionlib/pull/43.
  It is not actually correct - you can feasibly get feedback here before a new goal is confirmed. See `send_goal()`....
  ```
  def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
  # destroys the old goal handle
  self.stop_tracking_goal()
  ...
  self.gh = self.action_client.send_goal(goal, self._handle_transition, self._handle_feedback)
  ```
  and of course it will take more time on top of this for the server to actually process the incoming goal and confirm it. Meantime, it may have sent us feedback messages.
* Improved the const-correctness of some actionlib classes. (`#50 <https://github.com/ros/actionlib/issues/50>`_)
* Issue `#51 <https://github.com/ros/actionlib/issues/51>`_: Remove annoying debug messages that make useless to enable debug on Python nodes, as they overwhelm less spamming messages (`#54 <https://github.com/ros/actionlib/issues/54>`_)
* reduce change of unncessary exception on shutdown bu checking directly in before publishing for a shutdown (`#53 <https://github.com/ros/actionlib/issues/53>`_)
* Contributors: Blake Anderson, Daniel Stonier, Jorge Santos Simón, Mikael Arguedas, uliklank

1.11.6 (2016-06-22)
-------------------
* Python code cleanup (`#43 <https://github.com/ros/actionlib/issues/43>`_)
  * Cleaned up semicolons, indentation, spaces.
  * Removed unused local var after further confirmation of no risk of side effects.
* Contributors: Andrew Blakey

1.11.5 (2016-03-14)
-------------------
* update maintainer
* Merge pull request `#42 <https://github.com/ros/actionlib/issues/42>`_ from jonbinney/python3-compat
  Python 3 compatibility changes
* More readable iteration in state name lookup
* Update syntax for exception handling
* Iterate over dictionary in python3 compatible way
* Use absolute imports for python3 compatibility
* Merge pull request `#39 <https://github.com/ros/actionlib/issues/39>`_ from clearpathrobotics/action-fixup
  Minor improvements
* Enable UI feedback for preempt-requested goal in axserver.py
* Clean up axclient.py initialization to allow starting before actionserver, requires action type passed in
* Add hashes to ServerGoalHandle and ClientGoalHandles
* Contributors: Esteve Fernandez, Jon Binney, Mikael Arguedas, Paul Bovbel

1.11.4 (2015-04-22)
-------------------
* Initialize `execute_thread_` to NULL
* Contributors: Esteve Fernandez

1.11.3 (2014-12-23)
-------------------
* Increase queue sizes to match Python client publishers.
* Adjust size of client publishers in Python
* Contributors: Esteve Fernandez, Michael Ferguson

1.11.2 (2014-05-20)
-------------------
* Update python publishers to define queue_size.
* Use the correct queue for processing MessageEvents
* Contributors: Esteve Fernandez, Michael Ferguson, Nican

1.11.1 (2014-05-08)
-------------------
* Fix uninitialised `execute_thread_` member pointer
* Make rostest in CMakeLists optional
* Use catkin_install_python() to install Python scripts
* Contributors: Dirk Thomas, Esteve Fernandez, Jordi Pages, Lukas Bulwahn

1.11.0 (2014-02-13)
-------------------
* replace usage of __connection_header with MessageEvent (`#20 <https://github.com/ros/actionlib/issues/20>`_)

1.10.3 (2013-08-27)
-------------------
* Merged pull request `#15 <https://github.com/ros/actionlib/issues/15>`_
  Fixes a compile issue for actionlib headers on OS X

1.10.2 (2013-08-21)
-------------------
* separating ActionServer implementation into base class and ros-publisher-based class (`#11 <https://github.com/ros/actionlib/issues/11>`_)
* support CATKIN_ENABLE_TESTING
* add isValid to ServerGoalHandle (`#14 <https://github.com/ros/actionlib/issues/14>`_)
* make operators const (`#10 <https://github.com/ros/actionlib/issues/10>`_)
* add counting of connections to avoid reconnect problem when callbacks are invoked in different order (`#7 <https://github.com/ros/actionlib/issues/7>`_)
* fix deadlock in simple_action_server.py (`#4 <https://github.com/ros/actionlib/issues/4>`_)
* fix missing runtime destination for library (`#3 <https://github.com/ros/actionlib/issues/3>`_)

1.10.1 (2013-06-06)
-------------------
* fix location of library before installation (`#1 <https://github.com/ros/actionlib/issues/1>`_)

1.10.0 (2013-04-11)
-------------------
* define DEPRECATED only if not defined already
* modified dependency type of catkin to buildtool

1.9.11 (2012-12-13)
-------------------
* first public release for Groovy

1.8.7 (2012-06-14)
------------------
* add new CommState LOST
* added more missing dependencies

1.8.6 (2012-06-05)
------------------
* added missing dependencies

1.8.5 (2012-05-31)
------------------
* make axclient work base on topic name only

1.8.4 (2012-04-05)
------------------
* add missing axserver/axclient install

1.8.3 (2012-03-15)
------------------
* fix issue with locking in action server (`#5391 <https://code.ros.org/trac/ros-pkg/ticket/5391>`_)

1.8.2 (2012-02-29)
------------------
* update to newer catkin API

1.8.1 (2012-02-21)
------------------
* fix Python packaging

1.8.0 (2012-02-07)
------------------
* separated from common stack
* converted to use catkin
