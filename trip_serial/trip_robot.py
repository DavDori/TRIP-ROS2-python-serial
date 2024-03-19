from utils import saturate, deg2rad, rpm2radps
from encoder import Encoder
from enum import Enum
import numpy as np


class Speed(Enum):
    NONE = 0
    RPM = 1
    RADPS = 2
    DEGPS = 3
    

class TripRobot:
    def __init__(self, model, max_motor_rpm, encoders_ppr):
        self.Model = model
        self.max_motor_radps = rpm2radps(max_motor_rpm)
        self.EncoderLeft = Encoder(0, encoders_ppr)
        self.EncoderRight = Encoder(1, encoders_ppr)



    # return the desired string to control the vehicle motors
    def encodeMotorVel(self, left_vel, right_vel, measurement_unit):
        if(measurement_unit == Speed.RPM):
            left_vel_radps = rpm2radps(left_vel) 
            right_vel_radps = rpm2radps(right_vel)
            self.__encodeMotorVelRADPS(left_vel_radps, right_vel_radps)
        elif(measurement_unit == Speed.DEGPS):
            left_vel_radps = deg2rad(left_vel) 
            right_vel_radps = deg2rad(right_vel) 
            self.__encodeMotorVelRADPS(left_vel_radps, right_vel_radps)
        elif(measurement_unit == Speed.RADPS):
            self.__encodeMotorVelRADPS(left_vel, right_vel)
        elif(measurement_unit == Speed.NONE):
            # note that in this case, the motor is controlled in openloop
            self.__encodeMotorVelNorm(left_vel, right_vel)
        else:
            raise Exception('No valid measurement unit was selected for speed!')
    
    def __encodeMotorVelRADPS(self, left_vel_radps, right_vel_radps):
        left_vel_sat = saturate(left_vel_radps, self.max_motor_radps)
        right_vel_sat = saturate(right_vel_radps, self.max_motor_radps)

        encoded_motor_left = f"CSET,0,{left_vel_sat}"
        encoded_motor_right = f"CSET,1,{right_vel_sat}"
        
        # Concatenate the two strings with a delimiter
        encoded_string = encoded_motor_left + "\n" + encoded_motor_right + "\n"

        return encoded_string

    def __encodeMotorVelNorm(self, left_vel, right_vel):
        left_vel_sat = saturate(left_vel, 1.0)
        right_vel_sat = saturate(right_vel, 1.0)

        encoded_motor_left = f"MSET,0,{left_vel_sat}"
        encoded_motor_right = f"MSET,1,{right_vel_sat}"
        
        # Concatenate the two strings with a delimiter
        encoded_string = encoded_motor_left + "\n" + encoded_motor_right + "\n"

        return encoded_string
    

    
    def decodeEncoderData(self, msg):
        # expecting a message with structure: 
        # E<id>V<vel>,E<id>V<vel>\n
        # E<id>P<pulses>,E<id>P<pulses>\n
        if(msg == [] or not self.__isEncoderData(msg)):
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
        pulses = [self.EncoderLeft.getPulsesCount(), self.EncoderRight.getPulsesCount()]
        velocities = [self.EncoderLeft.getVelocity(), self.EncoderRight.getVelocity()]
        names = [self.EncoderLeft.getName(), self.EncoderRight.getName()]
        data = {'pulse_count': pulses, 'wheel_speed': velocities, 'names': names}
        return data

    def __isEncoderData(self, data_str):
        return data_str[0] == 'E'

    def __extractID(self, data_str):
        return data_str[1]
    
    