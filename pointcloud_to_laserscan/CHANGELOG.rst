^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package pointcloud_to_laserscan
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.1 (2021-10-19)
------------------
* Replace deprecated launch api (`#56 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/56>`_)
* Fix linting errors in the code (`#52 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/52>`_)
* Update README.md
* Contributors: Chris Lalancette, Daisuke Nishimatsu, Paul Bovbel

2.0.0 (2020-02-10)
------------------
* ROS 2 Migration (`#33 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/33>`_)
  * ROS 2 Migration
  Signed-off-by: Michel Hidalgo <michel@ekumenlabs.com>
* Contributors: Michel Hidalgo

1.4.1 (2019-08-30)
------------------
* LaserScan to PointCloud node + nodelet (`#28 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/28>`_)
* fix roslint (`#29 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/29>`_)
* Merge pull request `#20 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/20>`_ from ros-perception/range_max_check
  Add check for range_max
* Add check for range_max
* Contributors: Paul Bovbel, Rein Appeldoorn

1.4.0 (2017-11-14)
------------------
* Added inf_epsilon parameter that determines the value added to max range when use_infs parameter is set to false
* Merge pull request `#11 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/11>`_ from ros-perception/mikaelarguedas-patch-1
  update to use non deprecated pluginlib macro
* Migrate to package format 2; add roslint
* Add sane parameter defaults; fixup lint warnings
* update to use non deprecated pluginlib macro
* Contributors: Mikael Arguedas, Paul Bovbel, Prasanna Kannappan

1.3.1 (2017-04-26)
------------------
* Merge pull request `#4 <https://github.com/ros-perception/pointcloud_to_laserscan/issues/4>`_ from yoshimalucky/fix-miscalculation-in-angle-increment
  Fixed miscalculation in angle_increment in the launch files.
* fixed miscalculation in angle_increment in the launchfiles.
* Contributors: Paul Bovbel, yoshimalucky

1.3.0 (2015-06-09)
------------------
* Fix pointcloud to laserscan transform tolerance issues
* Move pointcloud_to_laserscan to new repository
* Contributors: Paul Bovbel

1.2.7 (2015-06-08)
------------------

* Cleanup pointcloud_to_laserscan launch files
* Contributors: Paul Bovbel

1.2.6 (2015-02-04)
------------------
* Fix default value for concurrency
* Fix multithreaded lazy pub sub
* Contributors: Paul Bovbel

1.2.5 (2015-01-20)
------------------
* Switch to tf_sensor_msgs for transform
* Set parameters in sample launch files to default
* Add tolerance parameter
* Contributors: Paul Bovbel

1.2.4 (2015-01-15)
------------------
* Remove stray dependencies
* Refactor with tf2 and message filters
* Remove roslaunch check
* Fix regressions
* Refactor to allow debug messages from node and nodelet
* Contributors: Paul Bovbel

1.2.3 (2015-01-10)
------------------
* add launch tests
* refactor naming and fix nodelet export
* set default target frame to empty
* clean up package.xml
* Contributors: Paul Bovbel

1.2.2 (2014-10-25)
------------------
* clean up package.xml
* Fix header reference
* Fix flow
* Fix pointer assertion
* Finalize pointcloud to laserscan
* Initial pointcloud to laserscan commit
* Contributors: Paul Bovbel
