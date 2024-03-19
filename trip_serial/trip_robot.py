from trip_serial.utils import saturate, degps2rpm, rpm2radps, radps2rpm
from trip_serial.encoder import Encoder
from enum import Enum


class Speed(Enum):
    NONE = 0
    RPM = 1
    RADPS = 2
    DEGPS = 3
    

class TripRobot:
    def __init__(self, model, max_motor_rpm, encoders_ppr):
        self.Model = model
        self.max_motor_rpm = max_motor_rpm
        self.EncoderLeft = Encoder(0, encoders_ppr, 'left')
        self.EncoderRight = Encoder(1, encoders_ppr, 'right')


    def encodeVehicleVel(self, lin_vel_mps, ang_vel_radps):
        self.Model.setVehicleVel(lin_vel_mps, ang_vel_radps)
        [omega_left, omega_right] = self.Model.getMotorsVel()

        return self.encodeMotorVel(omega_left, omega_right, Speed.RADPS)


    # return the desired string to control the vehicle motors
    def encodeMotorVel(self, left_vel, right_vel, measurement_unit):
        encoderd_cmd = ''
        if(measurement_unit == Speed.RPM):
            encoderd_cmd = self.__encodeMotorVelRPM(left_vel, right_vel)

        elif(measurement_unit == Speed.DEGPS):
            left_vel_rpm = degps2rpm(left_vel) 
            right_vel_rpm = degps2rpm(right_vel) 
            encoderd_cmd = self.__encodeMotorVelRPM(left_vel_rpm, right_vel_rpm)

        elif(measurement_unit == Speed.RADPS):
            left_vel_rpm = radps2rpm(left_vel) 
            right_vel_rpm = radps2rpm(right_vel) 
            encoderd_cmd = self.__encodeMotorVelRPM(left_vel_rpm, right_vel_rpm)

        elif(measurement_unit == Speed.NONE):
            # note that in this case, the motor is controlled in openloop
            encoderd_cmd = self.__encodeMotorVelNorm(left_vel, right_vel)
        else:
            raise Exception('No valid measurement unit was selected for speed!')
        return encoderd_cmd

    def __encodeMotorVelRPM(self, left_vel_rpm, right_vel_rpm):
        left_vel_sat = saturate(left_vel_rpm, self.max_motor_rpm)
        right_vel_sat = saturate(right_vel_rpm, self.max_motor_rpm)

        encoded_motor_left = f"CSET,0,%.2f" % left_vel_sat
        encoded_motor_right = f"CSET,1,%.2f" % right_vel_sat
        
        # Concatenate the two strings with a delimiter
        encoded_string = encoded_motor_left + "\n" + encoded_motor_right + "\n"

        return encoded_string

    def __encodeMotorVelNorm(self, left_vel, right_vel):
        left_vel_sat = saturate(left_vel, 1.0)
        right_vel_sat = saturate(right_vel, 1.0)

        encoded_motor_left = f"MSET,0,%.2f" % left_vel_sat
        encoded_motor_right = f"MSET,1,%.2f" % right_vel_sat
        
        # Concatenate the two strings with a delimiter
        encoded_string = encoded_motor_left + "\n" + encoded_motor_right + "\n"

        return encoded_string
    

    
    def decodeEncoderData(self, msg):
        # expecting a message with structure: 
        # E<id>V<vel>,E<id>V<vel>\n
        # E<id>P<pulses>,E<id>P<pulses>\n
        if(msg == '' or not self.__isEncoderData(msg)):
            return
        data = msg.split(',')
        for token in data:
            id = self.__extractID(token)
            self.__setEncoderData(id, token)
        
    def __setEncoderData(self, id, token):
        if(id == self.EncoderLeft.getID()):
            self.EncoderLeft.setData(token)
        elif(id == self.EncoderRight.getID()):
            self.EncoderRight.setData(token)

    def getEncoderData(self):
        angles = [self.EncoderLeft.getAngleRAD(), self.EncoderRight.getAngleRAD()]
        velocities = [self.EncoderLeft.getVelocity(), self.EncoderRight.getVelocity()]
        names = [self.EncoderLeft.getName(), self.EncoderRight.getName()]
        data = {'wheel_angle': angles, 'wheel_speed': velocities, 'names': names}
        return data

    def __isEncoderData(self, data_str):
        return data_str[0] == 'E'

    def __extractID(self, data_str):
        return data_str[1]
    
    