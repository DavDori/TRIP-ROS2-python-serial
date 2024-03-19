
class Encoder:
    def __init__(self, id, ppr, name):
        self.id = id
        self.ppr = ppr
        self.vel = 0.0
        self.pulses_count = 0
        self.name = name

    def getPulsesCount(self):
        return self.pulses_count
    
    def getVelocity(self):
        return self.vel
    
    def getID(self):
        return self.id
    
    def getPPR(self):
        return self.ppr
    
    def getName(self):
        return self.name
    
    def setVecloity(self, vel):
        self.vel = vel

    def setPulsesCount(self, pulses_count):
        self.pulses_count = pulses_count

    def setData(self, token):
        if(self.__isVelocityData(token)):
            self.vel = self.__parseEncoderVelocity(token)
        elif(self.__isPulseData(token)):
            self.pulses_count = self.__parseEncoderPulse(token)
    
    def __isVelocityData(self, token):
        return token[2] == 'V'
    
    def __isPulseData(self, token):
        return token[2] == 'P'
    
    def __parseEncoderVelocity(self, data_str):
        pass # TODO

    def __parseEncoderPulse(self, data_str):
        pass # TODO