#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# Author: furushchev <furushchev@jsk.imi.i.u-tokyo.ac.jp>
# Ported to ROS2 by [Your Name]

import ast
import operator as op
import traceback
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class RestrictedEvaluator:
    """Safe evaluation of mathematical expressions for joystick remapping."""

    def __init__(self):
        self.operators = {
            ast.Add: op.add,
            ast.Sub: op.sub,
            ast.Mult: op.mul,
            ast.Div: op.truediv,
            ast.BitXor: op.xor,
            ast.USub: op.neg,
        }
        self.functions = {
            'abs': abs,
            'max': max,
            'min': min,
        }

    def _reval_impl(self, node, variables):
        if isinstance(node, ast.Num):  # Number
            return node.n
        elif isinstance(node, ast.BinOp):  # Operators (+, -, *, /)
            return self.operators[type(node.op)](
                self._reval_impl(node.left, variables),
                self._reval_impl(node.right, variables),
            )
        elif isinstance(node, ast.UnaryOp):  # Unary (-x)
            return self.operators[type(node.op)](self._reval_impl(node.operand, variables))
        elif isinstance(node, ast.Call) and node.func.id in self.functions:  # Functions (min, max, abs)
            return self.functions[node.func.id](
                *[self._reval_impl(n, variables) for n in node.args]
            )
        elif isinstance(node, ast.Name) and node.id in variables:  # Variables
            return variables[node.id]
        elif isinstance(node, ast.Subscript) and node.value.id in variables:  # Array access
            var = variables[node.value.id]
            idx = node.slice.n
            if idx < len(var):
                return var[idx]
            raise IndexError(f"Variable '{node.value.id}' out of range: {idx} >= {len(var)}")
        else:
            raise TypeError(f"Unsupported operation: {node}")

    def reval(self, expr, variables):
        expr = str(expr)
        if len(expr) > 1000:
            raise ValueError("Expression too long (>1000 characters)")
        try:
            return self._reval_impl(ast.parse(expr, mode='eval').body, variables)
        except Exception:
            raise ValueError(f"Invalid expression: {expr}\n{traceback.format_exc()}")


class JoyRemap(Node):
    """ROS2 Node to remap joystick axes and buttons dynamically."""

    def __init__(self):
        super().__init__('joy_remap')
        self.evaluator = RestrictedEvaluator()

        # Declare parameters
        self.declare_parameter('mappings.buttons', [""])
        self.declare_parameter('mappings.axes', [""])
        # Load remapping rules from ROS2 parameters
        self.mappings = self.load_mappings()

        # Publishers and Subscribers
        self.publisher = self.create_publisher(Joy, 'joy_out', 10)
        self.subscription = self.create_subscription(
            Joy, 'joy_in', self.callback, 10)

        self.get_logger().info("JoyRemap Node Initialized")

        

    def load_mappings(self):
        """Load remapping configurations from parameters"""
        btn_param = self.get_parameter('mappings.buttons')
        axes_param = self.get_parameter('mappings.axes')
        print(btn_param)
        # Ensure the parameters are of type string array
        btn_remap = list(btn_param.get_parameter_value().string_array_value)
        axes_remap = list(axes_param.get_parameter_value().string_array_value)
        self.get_logger().info(f"Loaded remappings: {len(btn_remap)} buttons, {len(axes_remap)} axes")
        return {"buttons": btn_remap, "axes": axes_remap}

    def callback(self, in_msg):
        """Modify the joystick input based on mappings and publish"""
        out_msg = Joy()
        out_msg.header = in_msg.header
        out_msg.axes = [0.0] * len(self.mappings["axes"])
        out_msg.buttons = [0] * len(self.mappings["buttons"])
        in_dict = {"axes": in_msg.axes, "buttons": in_msg.buttons}

        for i, exp in enumerate(self.mappings["axes"]):
            try:
                out_msg.axes[i] = self.evaluator.reval(exp, in_dict)
            except Exception as e:
                self.get_logger().error(f"Axis remap error: {e}")

        for i, exp in enumerate(self.mappings["buttons"]):
            try:
                if self.evaluator.reval(exp, in_dict) > 0:
                    out_msg.buttons[i] = 1
            except Exception as e:
                self.get_logger().error(f"Button remap error: {e}")

        self.publisher.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoyRemap()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
