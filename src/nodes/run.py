#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from logger_collector.LogCollector import LogCollector

def main(args=None):
    rclpy.init(args=args)
    node = LogCollector()       # or whatever your class is called
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
