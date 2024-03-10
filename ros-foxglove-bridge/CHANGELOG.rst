^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package foxglove_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.7.6 (2024-02-26)
------------------
* Fix rolling builds (`#289 <https://github.com/foxglove/ros-foxglove-bridge/issues/289>`_)
* Remove dual ROS 1+2 devcontainer, remove ROS Galactic from the support matrix (`#285 <https://github.com/foxglove/ros-foxglove-bridge/issues/285>`_)
* Contributors: Hans-Joachim Krauch, John Hurliman

0.7.5 (2023-12-29)
------------------
* Add ROS 2 dependency for ament_index_cpp (`#281 <https://github.com/foxglove/ros-foxglove-bridge/issues/281>`_)
* Contributors: Chris Lalancette

0.7.4 (2023-12-14)
------------------
* Solved bug with incompatible QoS policies
* added explicit call to ParameterValue() to avoid clang error (`#277 <https://github.com/foxglove/ros-foxglove-bridge/issues/277>`_)
* Add iron release badge to readme (`#271 <https://github.com/foxglove/ros-foxglove-bridge/issues/271>`_)
* Contributors: Hans-Joachim Krauch, Ted

0.7.3 (2023-10-25)
------------------
* Fix `asset_uri_whitelist` regex backtracking issue, add more extensions (`#270 <https://github.com/foxglove/ros-foxglove-bridge/issues/270>`_)
* [ROS1] Fix callback accessing invalid reference to promise (`#268 <https://github.com/foxglove/ros-foxglove-bridge/issues/268>`_)
* Contributors: Hans-Joachim Krauch

0.7.2 (2023-09-12)
------------------
* Fix invalid pointers not being caught (`#265 <https://github.com/foxglove/ros-foxglove-bridge/issues/265>`_)
* Make ROS1 service type retrieval more robust (`#263 <https://github.com/foxglove/ros-foxglove-bridge/issues/263>`_)
* Contributors: Hans-Joachim Krauch

0.7.1 (2023-08-21)
------------------
* Communicate double / double array parameters with type info, explicitly cast when set from integer (`#256 <https://github.com/foxglove/ros-foxglove-bridge/issues/256>`_)
* Make ROS 2 smoke tests less flaky (`#260 <https://github.com/foxglove/ros-foxglove-bridge/issues/260>`_)
* Add debug config for ros2 smoke test (`#257 <https://github.com/foxglove/ros-foxglove-bridge/issues/257>`_)
* Handle client disconnection in message handler thread (`#259 <https://github.com/foxglove/ros-foxglove-bridge/issues/259>`_)
* Reduce smoke test flakiness (`#258 <https://github.com/foxglove/ros-foxglove-bridge/issues/258>`_)
* Server code improvements (`#250 <https://github.com/foxglove/ros-foxglove-bridge/issues/250>`_)
* Contributors: Hans-Joachim Krauch

0.7.0 (2023-07-12)
------------------
* Fix ROS2 launch file install rule not installing launch subfolder (`#243 <https://github.com/foxglove/ros-foxglove-bridge/issues/243>`_)
* Support building with boost asio (`#247 <https://github.com/foxglove/ros-foxglove-bridge/issues/247>`_)
* Avoid usage of tmpnam() for creating random filename (`#246 <https://github.com/foxglove/ros-foxglove-bridge/issues/246>`_)
* Implement ws-protocol's `fetchAsset` specification (`#232 <https://github.com/foxglove/ros-foxglove-bridge/issues/232>`_)
* Use `--include-eol-distros` for `rosdep` to fix melodic builds (`#244 <https://github.com/foxglove/ros-foxglove-bridge/issues/244>`_)
* Reduce logging severity for parameter retrieval logs (`#240 <https://github.com/foxglove/ros-foxglove-bridge/issues/240>`_)
* Contributors: Hans-Joachim Krauch, Micah Guttman

0.6.4 (2023-07-05)
------------------
* Assume publisher qos depth of 1 if the middleware reports the qos history as unknown (`#239 <https://github.com/foxglove/ros-foxglove-bridge/issues/239>`_)
* devcontainer: Use `--include-eol-distros` for `rosdep update` (`#237 <https://github.com/foxglove/ros-foxglove-bridge/issues/237>`_)
* Contributors: Hans-Joachim Krauch

0.6.3 (2023-06-16)
------------------
* Add iron build to CI (`#234 <https://github.com/foxglove/ros-foxglove-bridge/issues/234>`_)
* Fix QoS history being unknown when copied from existing publisher (`#233 <https://github.com/foxglove/ros-foxglove-bridge/issues/233>`_)
* Extract ROS 2 bridge header (`#228 <https://github.com/foxglove/ros-foxglove-bridge/issues/228>`_)
* Contributors: Hans-Joachim Krauch, Milan Vukov

0.6.2 (2023-05-11)
------------------
* Fix connection graph updates to due incorrect use of std::set_difference (`#226 <https://github.com/foxglove/ros-foxglove-bridge/issues/226>`_)
* Contributors: Ivan Nenakhov

0.6.1 (2023-05-05)
------------------
* Fix warning messages not being logged (`#224 <https://github.com/foxglove/ros-foxglove-bridge/issues/224>`_)
* Contributors: Hans-Joachim Krauch

0.6.0 (2023-05-04)
------------------
* Add support for nested parameters (ROS1) (`#221 <https://github.com/foxglove/ros-foxglove-bridge/issues/221>`_)
* Catch exceptions thrown in handler functions, send status to client (`#210 <https://github.com/foxglove/ros-foxglove-bridge/issues/210>`_)
* Fix unhandled xmlrpc exception (`#218 <https://github.com/foxglove/ros-foxglove-bridge/issues/218>`_)
* Add support for action topic and services (ROS2) (`#214 <https://github.com/foxglove/ros-foxglove-bridge/issues/214>`_)
* Add parameter to include hidden topics and services (ROS 2) (`#216 <https://github.com/foxglove/ros-foxglove-bridge/issues/216>`_)
* Add workaround for publishers not being cleaned up after they got destroyed (`#215 <https://github.com/foxglove/ros-foxglove-bridge/issues/215>`_)
* Fix error when compiling with C++20 (`#212 <https://github.com/foxglove/ros-foxglove-bridge/issues/212>`_)
* Devcontainer improvements (`#213 <https://github.com/foxglove/ros-foxglove-bridge/issues/213>`_)
* Add parameter for minimum subscription QoS depth (`#211 <https://github.com/foxglove/ros-foxglove-bridge/issues/211>`_)
* Log version and commit hash when node is started (`#209 <https://github.com/foxglove/ros-foxglove-bridge/issues/209>`_)
* Contributors: Hans-Joachim Krauch

0.5.3 (2023-03-31)
------------------
* Fix publishers being created with invalid QoS profile (`#205 <https://github.com/foxglove/ros-foxglove-bridge/issues/205>`_)
* Contributors: Hans-Joachim Krauch

0.5.2 (2023-03-29)
------------------
* Notify client when Server's send buffer limit has been reached (`#201 <https://github.com/foxglove/ros-foxglove-bridge/issues/201>`_)
* Add support for byte array params (`#199 <https://github.com/foxglove/ros-foxglove-bridge/issues/199>`_)
* Do not allow connection output buffer to exceed configured limit (`#196 <https://github.com/foxglove/ros-foxglove-bridge/issues/196>`_)
* Fix exception parameter not being used (`#194 <https://github.com/foxglove/ros-foxglove-bridge/issues/194>`_)
* Contributors: Hans-Joachim Krauch

0.5.1 (2023-03-09)
------------------
* Add more exception handling (`#191 <https://github.com/foxglove/ros-foxglove-bridge/issues/191>`_)
* [ROS1] Fix exception not being caught when retrieving service type  (`#190 <https://github.com/foxglove/ros-foxglove-bridge/issues/190>`_)
* Devcontainer: Use catkin tools, add build commands for ros1 (`#188 <https://github.com/foxglove/ros-foxglove-bridge/issues/188>`_)
* Contributors: Hans-Joachim Krauch

0.5.0 (2023-03-08)
------------------
* Add support for `schemaEncoding` field (`#186 <https://github.com/foxglove/ros-foxglove-bridge/issues/186>`_)
* Use QoS profile of existing publishers (if available) when creating new publishers (`#184 <https://github.com/foxglove/ros-foxglove-bridge/issues/184>`_)
* Make server more independent of given server configurations (`#185 <https://github.com/foxglove/ros-foxglove-bridge/issues/185>`_)
* Add parameter `client_topic_whitelist` for whitelisting client-published topics (`#181 <https://github.com/foxglove/ros-foxglove-bridge/issues/181>`_)
* Make server capabilities configurable (`#182 <https://github.com/foxglove/ros-foxglove-bridge/issues/182>`_)
* Fix action topic log spam (`#179 <https://github.com/foxglove/ros-foxglove-bridge/issues/179>`_)
* Remove (clang specific) compiler flag -Wmost (`#177 <https://github.com/foxglove/ros-foxglove-bridge/issues/177>`_)
* Improve the way compiler flags are set, use clang as default compiler (`#175 <https://github.com/foxglove/ros-foxglove-bridge/issues/175>`_)
* Avoid re-advertising existing channels when advertising new channels (`#172 <https://github.com/foxglove/ros-foxglove-bridge/issues/172>`_)
* Allow subscribing to connection graph updates (`#167 <https://github.com/foxglove/ros-foxglove-bridge/issues/167>`_)
* Contributors: Hans-Joachim Krauch

0.4.1 (2023-02-17)
------------------
* Run client handler functions in separate thread (`#165 <https://github.com/foxglove/ros-foxglove-bridge/issues/165>`_)
* Fix compilation error due to mismatched new-delete (`#163 <https://github.com/foxglove/ros-foxglove-bridge/issues/163>`_)
* Decouple server implementation (`#156 <https://github.com/foxglove/ros-foxglove-bridge/issues/156>`_)
* ROS2 parameter fixes (`#169 <https://github.com/foxglove/ros-foxglove-bridge/issues/169>`_)
* Fix program crash due to unhandled exception when creating publisher with invalid topic name (`#168 <https://github.com/foxglove/ros-foxglove-bridge/issues/168>`_)
* Contributors: Hans-Joachim Krauch

0.4.0 (2023-02-15)
------------------
* Update README with suggestion to build from source, minor fixes
* Do not build docker images, remove corresponding documentation (`#159 <https://github.com/foxglove/ros-foxglove-bridge/issues/159>`_)
* Add option to use permessage-deflate compression (`#152 <https://github.com/foxglove/ros-foxglove-bridge/issues/152>`_)
* Improve launch file documentation, add missing launch file arguments (`#158 <https://github.com/foxglove/ros-foxglove-bridge/issues/158>`_)
* Allow unsetting (deleting) parameters (`#145 <https://github.com/foxglove/ros-foxglove-bridge/issues/145>`_)
* Improve mutex usage (`#154 <https://github.com/foxglove/ros-foxglove-bridge/issues/154>`_)
* Add sessionId to serverInfo (`#153 <https://github.com/foxglove/ros-foxglove-bridge/issues/153>`_)
* Performance improvements (`#151 <https://github.com/foxglove/ros-foxglove-bridge/issues/151>`_)
* Add ROS2 support for calling server-advertised services (`#142 <https://github.com/foxglove/ros-foxglove-bridge/issues/142>`_)
* Add ROS1 support for calling server-advertised services (`#136 <https://github.com/foxglove/ros-foxglove-bridge/issues/136>`_)
* ROS2 smoke test: Increase default timeout 8->10 seconds (`#143 <https://github.com/foxglove/ros-foxglove-bridge/issues/143>`_)
* Fix flaky parameter test (noetic) (`#141 <https://github.com/foxglove/ros-foxglove-bridge/issues/141>`_)
* Always --pull when building docker images in the makefile (`#140 <https://github.com/foxglove/ros-foxglove-bridge/issues/140>`_)
* Fix failed tests not causing CI to fail (`#138 <https://github.com/foxglove/ros-foxglove-bridge/issues/138>`_)
* Fix setting `int` / `int[]` parameters not working (ROS 1) (`#135 <https://github.com/foxglove/ros-foxglove-bridge/issues/135>`_)
* Send ROS_DISTRO to clients via metadata field (`#134 <https://github.com/foxglove/ros-foxglove-bridge/issues/134>`_)
* Communicate supported encodings for client-side publishing (`#131 <https://github.com/foxglove/ros-foxglove-bridge/issues/131>`_)
* Fix client advertised channels not being updated on unadvertise (`#132 <https://github.com/foxglove/ros-foxglove-bridge/issues/132>`_)
* Add support for optional request id for `setParameter` operation (`#133 <https://github.com/foxglove/ros-foxglove-bridge/issues/133>`_)
* Fix exception when setting parameter to empty array (`#130 <https://github.com/foxglove/ros-foxglove-bridge/issues/130>`_)
* Fix wrong parameter field names being used (`#129 <https://github.com/foxglove/ros-foxglove-bridge/issues/129>`_)
* Add parameter support (`#112 <https://github.com/foxglove/ros-foxglove-bridge/issues/112>`_)
* Add throttled logging when send buffer is full (`#128 <https://github.com/foxglove/ros-foxglove-bridge/issues/128>`_)
* Contributors: Hans-Joachim Krauch, John Hurliman

0.3.0 (2023-01-04)
------------------
* Add launch files, add install instructions to README (`#125 <https://github.com/foxglove/ros-foxglove-bridge/issues/125>`_)
* Drop messages when connection send buffer limit has been reached (`#126 <https://github.com/foxglove/ros-foxglove-bridge/issues/126>`_)
* Remove references to galactic support from README (`#117 <https://github.com/foxglove/ros-foxglove-bridge/issues/117>`_)
* Add missing build instructions (`#123 <https://github.com/foxglove/ros-foxglove-bridge/issues/123>`_)
* Use a single reentrant callback group for all subscriptions (`#122 <https://github.com/foxglove/ros-foxglove-bridge/issues/122>`_)
* Fix clang compilation errors (`#119 <https://github.com/foxglove/ros-foxglove-bridge/issues/119>`_)
* Publish binary time data when `use_sim_time` parameter is `true` (`#114 <https://github.com/foxglove/ros-foxglove-bridge/issues/114>`_)
* Optimize Dockerfiles (`#110 <https://github.com/foxglove/ros-foxglove-bridge/issues/110>`_)
* Contributors: Hans-Joachim Krauch, Ruffin

0.2.2 (2022-12-12)
------------------
* Fix messages not being received anymore after unadvertising a client publication (`#109 <https://github.com/foxglove/ros-foxglove-bridge/issues/109>`_)
* Allow to whitelist topics via a ROS paramater (`#108 <https://github.com/foxglove/ros-foxglove-bridge/issues/108>`_)
* Contributors: Hans-Joachim Krauch

0.2.1 (2022-12-05)
------------------
* Fix compilation on platforms where size_t is defined as `unsigned int`
* Contributors: Hans-Joachim Krauch

0.2.0 (2022-12-01)
------------------

* Add support for client channels (`#66 <https://github.com/foxglove/ros-foxglove-bridge/issues/66>`_)
* Add smoke tests (`#72 <https://github.com/foxglove/ros-foxglove-bridge/issues/72>`_)
* Update package maintainers (`#70 <https://github.com/foxglove/ros-foxglove-bridge/issues/70>`_)
* [ROS2]: Fix messages not being received anymore after unsubscribing a topic (`#92 <https://github.com/foxglove/ros-foxglove-bridge/issues/92>`_)
* [ROS2]: Refactor node as a component (`#63 <https://github.com/foxglove/ros-foxglove-bridge/issues/63>`_)
* [ROS2]: Fix message definition loading for `.msg` or `.idl` files not located in `msg/` (`#95 <https://github.com/foxglove/ros-foxglove-bridge/issues/95>`_)
* [ROS1]: Mirror ROS 2 node behavior when `/clock`` topic is present (`#99 <https://github.com/foxglove/ros-foxglove-bridge/issues/99>`_)
* [ROS1]: Fix topic discovery function not being called frequently at startup (`#68 <https://github.com/foxglove/ros-foxglove-bridge/issues/68>`_)
* Contributors: Hans-Joachim Krauch, Jacob Bandes-Storch, John Hurliman

0.1.0 (2022-11-21)
------------------
* Initial release, topic subscription only
