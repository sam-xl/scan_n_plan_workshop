#!/usr/bin/python3

import math
import time
from collections import deque

import matplotlib.pyplot as plt
import rclpy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import ConnectivityException, ExtrapolationException, LookupException


class TFVelocityPlotter(Node):
    def __init__(self):
        super().__init__("tf_velocity_plotter")

        self.source_frame = "ur10e_base_link"
        self.target_frame = "tcp"

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Position-time window for smoothing
        self.window_size = 10  # number of samples to average over
        self.position_window = deque(maxlen=self.window_size)

        # Data for plotting
        self.times = deque(maxlen=700)
        self.velocities = deque(maxlen=700)
        self.start_time = time.time()

        # Matplotlib setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        (self.line,) = self.ax.plot([], [], label="Smoothed Linear Speed")
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Speed (m/s)")
        self.ax.set_title(
            f"Smoothed Speed of {self.target_frame} w.r.t. {self.source_frame}"
        )
        self.ax.legend()

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz TF lookup
        self.plot_timer = self.create_timer(0.1, self.update_plot)  # 10 Hz plot update

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                self.source_frame, self.target_frame, now
            )

            pos = trans.transform.translation
            timestamp = time.time()
            position = (pos.x, pos.y, pos.z)

            # Append to smoothing window
            self.position_window.append((timestamp, position))

            # Only compute velocity if we have enough samples
            if len(self.position_window) >= 2:
                t0, p0 = self.position_window[0]
                t1, p1 = self.position_window[-1]
                dt = t1 - t0
                if dt > 0:
                    dx = p1[0] - p0[0]
                    dy = p1[1] - p0[1]
                    dz = p1[2] - p0[2]
                    speed = math.sqrt(dx**2 + dy**2 + dz**2) / dt

                    self.times.append(timestamp - self.start_time)
                    self.velocities.append(speed)

        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warning("TF Lookup failed")
            return

    def update_plot(self):
        self.line.set_xdata(self.times)
        self.line.set_ydata(self.velocities)
        self.ax.relim()
        self.ax.autoscale_view()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


def main(args=None):
    rclpy.init(args=args)
    node = TFVelocityPlotter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()
