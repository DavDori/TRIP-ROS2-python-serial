import rclpy
import serial
import time 
from trip_serial.trip_robot import TripRobot, Speed
from rclpy.node import Node

from sensor_msgs.msg import JointState
from trip_serial.differential_drive import DifferentialDrive
from rclpy.exceptions import InvalidParameterValueException
from rclpy.signals import SignalHandlerOptions

class TripSerial(Node):

    def __init__(self):
        super().__init__('trip_base')
        self.publisher_ = self.create_publisher(JointState, 'enc', 10)
        self.subscription_ = self.create_subscription(
            JointState,
            'cmd_wheels',
            self.cmd_callback,
            10)
        
        self.declare_parameter('wheel_radius_m', 0.03)
        self.declare_parameter('wheel_distance_m', 0.16)
        self.declare_parameter('max_motor_rpm', 20)
        self.declare_parameter('enc_period_s', 0.02)
        self.declare_parameter('usb_port', '/dev/ttyACM0')
        self.declare_parameter('boudrate', 19200)
        self.declare_parameter('enable_timeout', False)
        self.declare_parameter('encoder_ppr', 300)
        
        self.__checkParams()

        r = self.get_parameter('wheel_radius_m').get_parameter_value().double_value
        d = self.get_parameter('wheel_distance_m').get_parameter_value().double_value
        max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        ppr = self.get_parameter('encoder_ppr').get_parameter_value().double_value
        
        enc_period_s = self.get_parameter('enc_period_s').get_parameter_value().double_value
        usb_port_name = self.get_parameter('usb_port').get_parameter_value().string_value
        boudrate = self.get_parameter('boudrate').get_parameter_value().integer_value
        en_timeout = self.get_parameter('enable_timeout').get_parameter_value().bool_value

        Model = DifferentialDrive(wheel_radius=r, wheel_distance=d, gearbox=1.0)
        self.Trip = TripRobot(Model, max_motor_rpm, ppr)
        if en_timeout:
            timeout = enc_period_s * 0.5
        else:
            timeout = 0.0

        self.timer_enc = self.create_timer(enc_period_s, self.timer_callback)
        self.ser = serial.Serial(
            port=usb_port_name,
            baudrate=boudrate,
            timeout=timeout,
            )
        # self.ser.open()
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.1)
        self.get_logger().info(f'Open communication on serial port {usb_port_name} with baudrate of {boudrate} .' , once=True)


    def cmd_callback(self, msg):
        if(len(msg) < 2):
            return
        left_vel = msg.velocity[0]
        right_vel = msg.velocity[1]

        encoderd_cmd = self.Trip.encodeMotorVel(left_vel, right_vel, Speed.RADPS)
        self.ser.write(encoderd_cmd)


    def timer_callback(self):
        self.ser.write('E'.encode('utf-8')) # send 'read' character
        ser_data = self.ser.readline()
        data_str = ser_data.decode("utf-8")
        self.Trip.decodeEncoderData(data_str)
        data = self.Trip.getEncoderData()

        msg = JointState()
        print(data)
        msg.position = data['wheel_angle']
        msg.velocity = data['wheel_speed']
        msg.name = data['names']

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'trip_base'
        self.publisher_.publish(msg)

    def __checkParams(self):
        r = self.get_parameter('wheel_radius_m').get_parameter_value().double_value
        d = self.get_parameter('wheel_distance_m').get_parameter_value().double_value
        max_motor_rpm = self.get_parameter('max_motor_rpm').get_parameter_value().double_value
        enc_period_s = self.get_parameter('enc_period_s').get_parameter_value().double_value

        if(r <= 0.0):
            raise InvalidParameterValueException('wheel_radius_m=',r,". Radius of the wheel is negative or 0.0! Change it to a positive value.")
        if(d <= 0.0):
            raise InvalidParameterValueException('wheel_distance_m=',d,". Distance between wheels is negative or 0.0! Change it to a positive value.")
        if(max_motor_rpm < 0.0):
            raise InvalidParameterValueException('max_motor_rpm=',max_motor_rpm,". Motor max speed is negative! Change it to a positive value.")
        if(enc_period_s < 0.0):
            raise InvalidParameterValueException('enc_period_s=',enc_period_s,". Period for encoder publisher is negative! Change it to a positive value.")

    def shutdown(self):
        self.ser.close()
    
def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    node = TripSerial()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except InvalidParameterValueException as e:
        node.get_logger().error("Caught InvalidParameterValueException: %s" % e)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
