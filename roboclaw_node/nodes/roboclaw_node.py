#!/usr/bin/env python
from math import pi, cos, sin
import sys
from rclpy import node
from rclpy.node import Node
from std_msgs.msg import String
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from diagnostic_updater import DiagnosticTask, DiagnosticStatusWrapper, Updater
from geometry_msgs.msg import TransformStamped
from roboclaw_driver.robocSlaw_driver import Roboclaw
import rclpy
import threading
import tf2_ros
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import Quaternion, Twist
from nav_msgs.msg import Odometry

__author__ = "bwbazemore@uga.edu (Brad Bazemore)"

# TODO need to find some better was of handling OSerror 11 or preventing it, any ideas?

class EncoderOdom:
    def __init__(self, ticks_per_meter, base_width):
        self.TICKS_PER_METER = ticks_per_meter
        self.BASE_WIDTH = base_width
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0           # M1=left
        self.last_enc_right = 0          # M2=right
        self.last_enc_time = node.get_clock().now()

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        left_ticks = enc_left - self.last_enc_left
        right_ticks = enc_right - self.last_enc_right
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = rclpy.get_clock().now() # have confusion on this as i have to create an rclpy.init() node to grab the time
        d_time = current_time.nanoseconds - self.last_enc_time.nanoseconds
        self.last_enc_time = current_time

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            d_theta = (dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 0.000001:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / d_time
            vel_theta = d_theta / d_time

        return vel_x, vel_theta

    def update_publish(self, enc_left, enc_right):
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            self.get_logger().error("Ignoring left encoder jump: cur %d, last %d" % (enc_left, self.last_enc_left))
        elif abs(enc_right - self.last_enc_right) > 20000:
            self.get_logger().error("Ignoring right encoder jump: cur %d, last %d" % (enc_right, self.last_enc_right))
        else:
            vel_x, vel_theta = self.update(enc_left, enc_right)
            self.publish_odom(self.cur_x, self.cur_y, self.cur_theta, vel_x, vel_theta)

    def publish_odom(self, cur_x, cur_y, cur_theta, vx, vth):
        quat = tf2_ros.transforms3d.euler.euler2quat(0, 0, cur_theta)
        current_time = node.get_clock().now().to_msg()

        br = TransformBroadcaster(node)      
        tfs = TransformStamped()
        tfs.header.stamp = current_time.to_msg()

        tfs.header.frame_id="odom"
        tfs._child_frame_id = "base_footprint"

        tfs.transform.rotation.x = quat[0]
        tfs.transform.rotation.y = quat[1]
        tfs.transform.rotation.z = quat[2]
        tfs.transform.rotation.w = quat[3]
        br.sendTransform(tfs) 

        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = 'odom'

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(*quat)

        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.01

        odom.child_frame_id = 'base_footprint'
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        self.odom_pub.publish(odom)

    def shutdown(self):
        # perform any cleanup or shutdown operation # Have to check about this thing as well Faraz
        pass

class Roboclaw_node(Node):
    def __init__(self):
        super().__init__("roboclaw_node")

        self.ERRORS = {0x0000: (DiagnosticStatus.OK, "Normal"),
                       0x0001: (DiagnosticStatus.WARN, "M1 over current"),
                       0x0002: (DiagnosticStatus.WARN, "M2 over current"),
                       0x0004: (DiagnosticStatus.ERROR, "Emergency Stop"),
                       0x0008: (DiagnosticStatus.ERROR, "Temperature1"),
                       0x0010: (DiagnosticStatus.ERROR, "Temperature2"),
                       0x0020: (DiagnosticStatus.ERROR, "Main batt voltage high"),
                       0x0040: (DiagnosticStatus.ERROR, "Logic batt voltage high"),
                       0x0080: (DiagnosticStatus.ERROR, "Logic batt voltage low"),
                       0x0100: (DiagnosticStatus.WARN, "M1 driver fault"),
                       0x0200: (DiagnosticStatus.WARN, "M2 driver fault"),
                       0x0400: (DiagnosticStatus.WARN, "Main batt voltage high"),
                       0x0800: (DiagnosticStatus.WARN, "Main batt voltage low"),
                       0x1000: (DiagnosticStatus.WARN, "Temperature1"),
                       0x2000: (DiagnosticStatus.WARN, "Temperature2"),
                       0x4000: (DiagnosticStatus.OK, "M1 home"),
                       0x8000: (DiagnosticStatus.OK, "M2 home")}

        rclpy.init(args=sys.argv)
        node = rclpy.create_node("roboclaw_node")
        rclpy.on_shutdown(self.shutdown)
        node.get_logger().info("Connecting to roboclaw")
        dev_name = self.get_parameter("dev").get_parameter_value().string_value
        baud_rate = int(self.get_parameter("baud").get_parameter_value().string_value) # have to check this and above line as well  "115200")

        self.address = int(self.get_parameter("address").get_parameter_value().string_value) # int(rospy.get_param("~address", "128"))
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            raise RuntimeError("Address out of range")

        self.roboclaw = Roboclaw(dev_name, baud_rate)
        # TODO need someway to check if address is correct
        try:
            self.roboclaw.Open()
        except Exception as e:
            self.get_logger().fatal("Could not connect to Roboclaw")
            self.get_logger().debug(str(e))
            raise RuntimeError("Could not connect to Roboclaw")

        self.updater = Updater()
        self.updater.setHardwareID("Roboclaw")
        self.updater.add(DiagnosticTask("Vitals", self.check_vitals))
        try:
            version = self.roboclaw.ReadVersion(self.address)
        except Exception as e:
            self.get_logger().warn("Problem getting roboclaw version")
            self.get_logger().debug(str(e))
            pass

        if not version[0]:
            self.get_logger().warn("Could not get version from roboclaw")
        else:
            self.get_logger().debug(repr(version[1]))

        self.roboclaw.SpeedM1M2(self.address, 0, 0)
        self.roboclaw.ResetEncoders(self.address)

        self.MAX_SPEED = float(self.get_parameter("max_speed").get_parameter_value().string_value)  #("~max_speed", "2.0"))
        self.TICKS_PER_METER = float(self.get_parameter("ticks_per_meter").get_parameter_value().string_value) #("~ticks_per_meter", "4342.2"))
        self.BASE_WIDTH = float(self.get_parameter("base_width").get_parameter_value().string_value) #("~base_width", "0.315"))
        self.INVERT_MOTOR_DIRECTION = self.get_parameter("invert_motor_direction").get_parameter_value().bool_value#("~invert_motor_direction", False)
        self.FLIP_LEFT_AND_RIGHT_MOTORS = self.get_parameter("flip_left_and_right_motors").get_parameter_value().bool_value #("~flip_left_and_right_motors", False)
        
        self.encodm = EncoderOdom(self.TICKS_PER_METER, self.BASE_WIDTH)
        self.last_set_speed_time = node.get_clock().now()

        self.create_subscription(Twist, "base/cmd_vel", self.cmd_vel_callback, 1)

        rclpy.sleep(1)

        self.get_logger().debug("dev %s", dev_name)
        self.get_logger().debug("baud %d", baud_rate)
        self.get_logger().debug("address %d", self.address)
        self.get_logger().debug("max_speed %f", self.MAX_SPEED)
        self.get_logger().debug("ticks_per_meter %f", self.TICKS_PER_METER)
        self.get_logger().debug("base_width %f", self.BASE_WIDTH)
        self.get_logger().debug("invert_motor_direction %f", self.INVERT_MOTOR_DIRECTION)
        self.get_logger().debug("flip_left_and_right_motors %f", self.FLIP_LEFT_AND_RIGHT_MOTORS)

    def run(self):
        self.get_logger().info("Starting motor drive")
        r_time = self.create_rate(10)
        while rclpy.ok(): #not rospy.is_shutdown():

            if (node.get_clock().now() - self.last_set_speed_time).to_sec() > rclpy.time.Duration(seconds=0.3):
                self.get_logger().debug("Did not get comand for 1 second, stopping")
                try:
                    self.roboclaw.ForwardM1(self.address, 0)
                    self.roboclaw.ForwardM2(self.address, 0)
                except OSError as e:
                    self.get_logger().error("Could not stop")
                    self.get_logger().debug(str(e))

            # TODO need find solution to the OSError11 looks like sync problem with serial
            status_left, enc_left, crc_left = None, None, None
            status_right, enc_right, crc_right = None, None, None

            try:
                status_left, enc_left, crc_left = self.roboclaw.ReadEncM1(self.address)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn("ReadEncM1 OSError: %d", e.errno)
                self.get_logger().debug(str(e))

            try:
                status_right, enc_right, crc_right = self.roboclaw.ReadEncM2(self.address)
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn("ReadEncM2 OSError: %d", e.errno)
                self.get_logger().debug(str(e))

            # if (enc1 in locals()) and (enc2 in locals()):
            if self.INVERT_MOTOR_DIRECTION:
                enc_left = -enc_left
                enc_right = -enc_right

            if self.FLIP_LEFT_AND_RIGHT_MOTORS:
                enc_left, enc_right = enc_right, enc_left

            try:
                self.get_logger().debug(" Encoders %d %d" % (enc_left, enc_right))
                self.encodm.update_publish(enc_left, enc_right)  # update_publish expects enc_left enc_right
                self.updater.update()
            except:
                print("problems reading encoders")

            r_time.sleep() ##didnot change this to rate.sleep() bcx r_time is already defined

    def cmd_vel_callback(self, twist):
        self.last_set_speed_time = node.get_clock().now()

        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        vel_right = linear_x + twist.angular.z * self.BASE_WIDTH / 2.0  # m/s
        vel_left = linear_x - twist.angular.z * self.BASE_WIDTH / 2.0

        if self.INVERT_MOTOR_DIRECTION:
            vel_right = -vel_right
            vel_left = -vel_left

        if self.FLIP_LEFT_AND_RIGHT_MOTORS:
            vel_left, vel_right = vel_right, vel_left

        left_ticks = int(vel_left * self.TICKS_PER_METER)
        right_ticks = int(vel_right * self.TICKS_PER_METER)  # ticks/s

        self.get_logger().debug("left_ticks:%d right_ticks: %d", left_ticks, right_ticks)

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if left_ticks is 0 and right_ticks is 0:
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
            else:
                self.roboclaw.SpeedM1M2(self.address, left_ticks, right_ticks)
        except OSError as e:
            self.get_logger().warn("SpeedM1M2 OSError: %d", e.errno)
            self.get_logger().debug(str(e))

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = self.roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(str(e))
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add("Main Batt V:", str(self.roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10))
            stat.add("Logic Batt V:", str(self.roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10))
            stat.add("Temp1 C:", str(self.roboclaw.ReadTemp(self.address)[1] / 10))
            stat.add("Temp2 C:", str(self.roboclaw.ReadTemp2(self.address)[1] / 10))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: %d", e.errno)
            self.get_logger().debug(str(e))
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self):
        self.get_logger().info("Shutting down")
        try:
            self.roboclaw.ForwardM1(self.address, 0)
            self.roboclaw.ForwardM2(self.address, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                self.roboclaw.ForwardM1(self.address, 0)
                self.roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().debug(str(e))


if __name__ == "__main__":
    try:
        rclpy.init()
        node = Roboclaw_node()
        node.run()
    except rclpy.exception.ROSInterruptException:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("Quiting")
