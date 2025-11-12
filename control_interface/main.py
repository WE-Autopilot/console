import rclpy

from control_ui import AP1DebugUI, AP1SystemInterfaceNode

# main time bbgl
def main(args=None):
    rclpy.init(args=args)
    ros_node = AP1SystemInterfaceNode()
    app = AP1DebugUI(ros_node)

    try:
        # run
        app.run()
    finally:
        # teardown
        ros_node.destroy_node()
        rclpy.shutdown()

# bismAllah
if __name__ == '__main__':
    main()