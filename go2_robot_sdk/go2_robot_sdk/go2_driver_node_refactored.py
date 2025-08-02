# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

"""
Refactored entry point for Go2 Driver Node
with clean architecture
"""

import asyncio
import threading
import rclpy
from rclpy.executors import SingleThreadedExecutor

from .presentation.go2_driver_node import Go2DriverNode


async def run_robot_connections(node: Go2DriverNode):
    """Start robot connections"""
    try:
        # Connect to robots
        await node.connect_robots()

        # Start control loops for each robot
        tasks = []
        for i in range(len(node.config.robot_ip_list)):
            robot_id = str(i)
            task = asyncio.create_task(node.run_robot_control_loop(robot_id))
            tasks.append(task)

        # Wait for all tasks to complete
        await asyncio.gather(*tasks)

    except Exception as e:
        node.get_logger().error(f"Error in robot connections: {e}")
        raise


async def spin_node(node: Go2DriverNode):
    """Run ROS2 node in a separate thread"""
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    
    def spin_thread():
        try:
            executor.spin()
        except Exception as e:
            node.get_logger().error(f"Error in ROS2 spin: {e}")
        finally:
            executor.shutdown()

    # Start in a separate thread
    thread = threading.Thread(target=spin_thread, daemon=True)
    thread.start()

    try:
        # Wait for node to finish
        while rclpy.ok():
            await asyncio.sleep(0.1)
    finally:
        thread.join(timeout=1.0)


async def main_async():
    """Main asynchronous function"""
    # Initialize ROS2
    rclpy.init()

    try:
        # Create node
        node = Go2DriverNode()
        
        # Run ROS2 node and robot connections in parallel
        ros_task = asyncio.create_task(spin_node(node))
        robot_task = asyncio.create_task(run_robot_connections(node))

        # Wait for any task to complete
        done, pending = await asyncio.wait(
            [ros_task, robot_task],
            return_when=asyncio.FIRST_COMPLETED
        )

        # Cancel unfinished tasks
        for task in pending:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass

        # Check results of completed tasks
        for task in done:
            try:
                await task
            except Exception as e:
                node.get_logger().error(f"Task failed: {e}")

    except KeyboardInterrupt:
        print("Shutting down due to keyboard interrupt")
    except Exception as e:
        print(f"Fatal error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Resource cleanup
        try:
            # Disconnect from robots
            if 'node' in locals() and hasattr(node, 'webrtc_adapter'):
                for robot_id in node.webrtc_adapter.connections:
                    await node.webrtc_adapter.disconnect(robot_id)
            
            # Close unfinished tasks
            tasks = [t for t in asyncio.all_tasks() if not t.done()]
            if tasks:
                for task in tasks:
                    task.cancel()
                await asyncio.gather(*tasks, return_exceptions=True)

        except Exception as e:
            print(f"Error during cleanup: {e}")
        finally:
            rclpy.shutdown()


def main():
    """Entry point"""
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        print("\nNode terminated by keyboard interrupt")
    except Exception as e:
        print(f"Fatal error in main: {e}")
        import traceback
        traceback.print_exc()


if __name__ == '__main__':
    main() 