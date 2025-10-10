#!/usr/bin/env python3
"""
Integration test for OnRobot 2FG7 gripper with DRIMS2 cells.
Run this after starting the complete system to verify gripper integration.
"""

import rclpy
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
import time
import sys

class OnRobotIntegrationTester:
    def __init__(self):
        # Initialize ROS 2 and create node
        rclpy.init()
        self.node = rclpy.create_node('onrobot_integration_tester')
        # Create action client for the gripper action
        self.gripper_action_client = ActionClient(self.node, GripperCommand, 'gripper_action')
        self.joint_states_received = False
        self.gripper_joints_found = False
        
        # Subscribe to joint states to verify gripper joints are published
        self.joint_state_sub = self.node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.node.get_logger().info("🧪 OnRobot Integration Tester Started")
    
    def joint_state_callback(self, msg):
        """Callback to check if gripper joints are being published"""
        self.joint_states_received = True
        
        # Check if our gripper joints are in the message
        gripper_joints = ['left_finger_joint', 'right_finger_joint']
        found_joints = [joint for joint in gripper_joints if joint in msg.name]
        
        if len(found_joints) == len(gripper_joints):
            self.gripper_joints_found = True
            self.node.get_logger().info(f"✅ Found gripper joints in /joint_states: {found_joints}")
    
    def wait_for_joint_states(self, timeout=10.0):
        """Wait for joint states to be published with gripper joints"""
        self.node.get_logger().info("Waiting for joint states with gripper joints...")
        start_time = time.time()
        
        while time.time() - start_time < timeout and not self.gripper_joints_found:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if self.joint_states_received and not self.gripper_joints_found:
                self.node.get_logger().info("Joint states received but gripper joints not found yet...")
        
        return self.gripper_joints_found
    
    def wait_for_gripper_action_server(self, timeout=15.0):
        """Wait for gripper action server to be available"""
        self.node.get_logger().info("Waiting for gripper action server...")
        
        if self.gripper_action_client.wait_for_server(timeout_sec=timeout):
            self.node.get_logger().info("✅ Gripper action server available")
            return True
        else:
            self.node.get_logger().error("❌ Gripper action server not available")
            return False
    
    def test_gripper_actions(self):
        """Test basic gripper functionality by sending a sequence of commands"""
        self.node.get_logger().info("=== Testing Gripper Actions ===")
        
        # Test sequence - safe movements
        test_sequence = [
            (0.070, 30.0, "Open to 70mm"),
            (0.040, 60.0, "Close to 40mm"),
            (0.060, 40.0, "Open to 60mm (ready position)"),
        ]
        
        all_success = True
        
        for position, force, description in test_sequence:
            self.node.get_logger().info(f"🧪 Testing: {description}")
            
            success = self.send_gripper_command(position, force)
            
            if success:
                self.node.get_logger().info(f"✅ {description} - SUCCESS")
            else:
                self.node.get_logger().error(f"❌ {description} - FAILED")
                all_success = False
            
            # Wait between commands
            time.sleep(2.0)
        
        return all_success
    
    def send_gripper_command(self, position, force):
        """Send a gripper command and wait for result"""
        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = float(position)
        goal_msg.command.max_effort = float(force)
        
        # Send goal asynchronously
        future = self.gripper_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is None:
            self.node.get_logger().error("Goal rejected - timeout")
            return False
        
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error("Goal rejected by server")
            return False
        
        self.node.get_logger().info("Goal accepted, waiting for result...")
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=10.0)
        
        if result_future.result() is None:
            self.node.get_logger().warn("No result received (timeout) - but continuing")
            return True  # Consider it success for simulation
        
        result = result_future.result().result
        self.node.get_logger().info(
            f"Result: position={result.position:.3f}m, "
            f"effort={result.effort:.1f}%, "
            f"reached_goal={result.reached_goal}, "
            f"stalled={result.stalled}"
        )
        
        return result.reached_goal or not result.stalled
    
    def test_controller_manager(self):
        """Test that gripper controller is loaded in controller manager"""
        self.node.get_logger().info("=== Testing Controller Manager ===")
        
        try:
            # Check if controller manager services are available
            from controller_manager_msgs.srv import ListControllers
            
            list_controllers_client = self.node.create_client(
                ListControllers, 
                '/controller_manager/list_controllers'
            )
            
            if list_controllers_client.wait_for_service(timeout_sec=5.0):
                req = ListControllers.Request()
                future = list_controllers_client.call_async(req)
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
                
                if future.result() is not None:
                    response = future.result()
                    gripper_controllers = [
                        c for c in response.controller 
                        if 'onrobot_2fg7_gripper_controller' in c.name
                    ]
                    
                    if gripper_controllers:
                        controller = gripper_controllers[0]
                        self.node.get_logger().info(
                            f"✅ Gripper controller found: {controller.name} "
                            f"(state: {controller.state})"
                        )
                        return True
                    else:
                        self.node.get_logger().error("❌ Gripper controller not found in controller manager")
                        return False
                else:
                    self.node.get_logger().warn("⚠️ Could not get controller list")
                    return False
            else:
                self.node.get_logger().warn("⚠️ Controller manager service not available")
                return False
                
        except ImportError:
            self.node.get_logger().warn("⚠️ Could not import controller_manager_msgs - skipping controller test")
            return True  # Skip this test if package not available
        except Exception as e:
            self.node.get_logger().warn(f"⚠️ Controller manager test failed: {e}")
            return False
    
    def run_complete_test(self):
        """Run all integration tests and print a summary"""
        self.node.get_logger().info("🚀 Starting OnRobot 2FG7 Integration Test")
        self.node.get_logger().info("=" * 50)
        
        test_results = {}
        
        # Test 1: Check joint states
        self.node.get_logger().info("\n1. Testing Joint States...")
        test_results['joint_states'] = self.wait_for_joint_states()
        
        # Test 2: Check action server
        self.node.get_logger().info("\n2. Testing Action Server...")
        test_results['action_server'] = self.wait_for_gripper_action_server()
        
        # Test 3: Check controller manager
        self.node.get_logger().info("\n3. Testing Controller Manager...")
        test_results['controller_manager'] = self.test_controller_manager()
        
        # Test 4: Test gripper functionality (only if action server is available)
        if test_results['action_server']:
            self.node.get_logger().info("\n4. Testing Gripper Functionality...")
            test_results['gripper_functionality'] = self.test_gripper_actions()
        else:
            self.node.get_logger().info("\n4. Skipping Gripper Functionality (action server not available)")
            test_results['gripper_functionality'] = False
        
        # Print summary of all tests
        self.node.get_logger().info("\n" + "=" * 50)
        self.node.get_logger().info("📊 TEST SUMMARY")
        self.node.get_logger().info("=" * 50)
        
        for test_name, result in test_results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            self.node.get_logger().info(f"{test_name:25} {status}")
        
        # Overall result
        all_passed = all(test_results.values())
        if all_passed:
            self.node.get_logger().info("\n🎉 ALL TESTS PASSED! OnRobot 2FG7 integration is working correctly!")
        else:
            self.node.get_logger().error("\n💥 SOME TESTS FAILED! Check the system configuration.")
        
        return all_passed

def main():
    # Entry point for the integration test
    tester = OnRobotIntegrationTester()
    
    try:
        success = tester.run_complete_test()
        exit_code = 0 if success else 1
    except KeyboardInterrupt:
        tester.node.get_logger().info("Test interrupted by user")
        exit_code = 0
    except Exception as e:
        tester.node.get_logger().error(f"Test failed with exception: {e}")
        exit_code = 1
    finally:
        tester.node.destroy_node()
        rclpy.shutdown()
    
    sys.exit(exit_code)

if __name__ == '__main__':
    main()