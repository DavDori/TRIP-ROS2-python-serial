

class DifferentialDrive:
    def __init__(self, wheel_radius, wheel_distance, gearbox):
        self.wheel_radius = wheel_radius
        self.wheel_distance = wheel_distance
        # the gearbox ratio is measured as the times the torque is
        # multiplied from the motor to the wheel
        self.gearbox_ratio = gearbox
        self.right_motor_omega_radps = 0.0
        self.left_motor_omega_radps = 0.0

    def getMotorsSpeed(self):
        return [self.left_motor_omega_radps, self.right_motor_omega_radps]
    
    def getWheelsSpeed(self):
        tau = self.gearbox_ratio
        return [self.left_motor_omega_radps / tau, self.right_motor_omega_radps / tau]
    
    # use the kinematic model of the differential drive model
    # to convert velocities of the motor into velocities of 
    # the vehicle
    def getVehicleSpeed(self):
        tau = self.gearbox_ratio
        r =  self.wheel_radius
        omega_r = self.right_motor_omega_radps
        omega_l = self.left_motor_omega_radps
        d = self.wheel_distance

        lin_vel = (omega_r + omega_l) * r * 0.5 / tau
        ang_vel = (omega_r - omega_l) * r / (tau * d)
        return [lin_vel, ang_vel]

    def setMotorsSpeed(self, left_motor_omega_radps, right_motor_omega_radps):
        self.left_motor_omega_radps = left_motor_omega_radps
        self.right_motor_omega_radps = right_motor_omega_radps

    def setWheelsSpeed(self, left_wheel_omega_radps, right_wheel_omega_radps):
        tau = self.gearbox_ratio

        self.left_motor_omega_radps = left_wheel_omega_radps * tau
        self.right_motor_omega_radps = right_wheel_omega_radps * tau

    # use the kinematic model of the differential drive model
    # to convert velocities of the vehicle into velocities of 
    # the motors
    def setVehicleSpeed(self, lin_vel_mps, ang_vel_radps):
        r =  self.wheel_radius
        d = self.wheel_distance

        left_wheel_omega_radps = (lin_vel_mps - d * 0.5 * ang_vel_radps) / r
        right_wheel_omega_radps = (lin_vel_mps + d * 0.5 * ang_vel_radps) / r

        self.setWheelsSpeed(left_wheel_omega_radps, right_wheel_omega_radps)