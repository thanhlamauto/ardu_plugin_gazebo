#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import FluidPressure, Imu, MagneticField, NavSatFix
from visualization_msgs.msg import Marker, MarkerArray


class SensorStatusMarkers(Node):
    def __init__(self):
        super().__init__("sensor_status_markers")
        self.pressure = None
        self.magnetic_field = None
        self.navsat = None
        self.imu = None

        self.create_subscription(
            FluidPressure, "/sensor_suite/air_pressure", self.pressure_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            MagneticField, "/sensor_suite/magnetic_field", self.magnetic_field_callback, qos_profile_sensor_data
        )
        self.create_subscription(NavSatFix, "/sensor_suite/navsat", self.navsat_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, "/sensor_suite/imu", self.imu_callback, qos_profile_sensor_data)

        self.publisher = self.create_publisher(MarkerArray, "/sensor_suite/status_markers", 10)
        self.create_timer(0.1, self.publish_markers)

    def pressure_callback(self, msg):
        self.pressure = msg

    def magnetic_field_callback(self, msg):
        self.magnetic_field = msg

    def navsat_callback(self, msg):
        self.navsat = msg

    def imu_callback(self, msg):
        self.imu = msg

    def text_marker(self, marker_id, text, y, red, green, blue):
        marker = Marker()
        marker.header.frame_id = "sensor_suite_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "sensor_status"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = 0.0
        marker.pose.position.y = y
        marker.pose.position.z = 1.2
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.22
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0
        marker.text = text
        return marker

    def arrow_marker(self, marker_id, namespace, vector, red, green, blue):
        marker = Marker()
        marker.header.frame_id = "sensor_suite_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale.x = 0.035
        marker.scale.y = 0.08
        marker.scale.z = 0.12
        marker.color.r = red
        marker.color.g = green
        marker.color.b = blue
        marker.color.a = 1.0

        magnitude = math.sqrt(sum(component * component for component in vector))
        unit = (0.0, 0.0, 0.0) if magnitude == 0.0 else tuple(component / magnitude for component in vector)
        marker.points = [Point(x=0.0, y=0.0, z=0.35), Point(x=unit[0], y=unit[1], z=0.35 + unit[2])]
        return marker

    def publish_markers(self):
        markers = MarkerArray()

        pressure_text = "Barometer: waiting"
        if self.pressure is not None:
            pressure_text = f"Barometer: {self.pressure.fluid_pressure / 100.0:.2f} hPa"
        markers.markers.append(self.text_marker(0, pressure_text, -0.75, 0.9, 0.9, 0.2))

        gps_text = "NavSat: waiting"
        if self.navsat is not None:
            gps_text = f"GPS: {self.navsat.latitude:.6f}, {self.navsat.longitude:.6f}"
        markers.markers.append(self.text_marker(1, gps_text, -0.25, 0.2, 0.9, 0.9))

        magnetic_text = "Magnetometer: waiting"
        if self.magnetic_field is not None:
            field = self.magnetic_field.magnetic_field
            microtesla = 1e6 * math.sqrt(field.x * field.x + field.y * field.y + field.z * field.z)
            magnetic_text = f"Magnetometer: {microtesla:.2f} uT"
            markers.markers.append(
                self.arrow_marker(2, "magnetic_field", (field.x, field.y, field.z), 0.9, 0.2, 0.9)
            )
        markers.markers.append(self.text_marker(2, magnetic_text, 0.25, 0.9, 0.3, 0.9))

        imu_text = "IMU: waiting"
        if self.imu is not None:
            acceleration = self.imu.linear_acceleration
            magnitude = math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
            imu_text = f"IMU acceleration: {magnitude:.3f} m/s^2"
            markers.markers.append(
                self.arrow_marker(3, "imu_acceleration", (acceleration.x, acceleration.y, acceleration.z), 0.2, 0.9, 0.2)
            )
        markers.markers.append(self.text_marker(3, imu_text, 0.75, 0.3, 1.0, 0.3))

        self.publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = SensorStatusMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
