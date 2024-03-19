import numpy as np

class Encoder:
    def __init__(self, id, ppr, name):
        self.id = id
        self.ppr = ppr
        self.vel = 0.0
        self.pulses_count = 0.0
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
    
    def getAngleRAD(self):
        return 2 * np.pi * self.pulses_count / self.ppr

    def setData(self, token):
        data = self.__extractData(token)
        if(self.__isVelocityData(token)):
            self.vel = data
        elif(self.__isPulseData(token)):
            self.pulses_count = data
    
    def __isVelocityData(self, token):
        return token[2] == 'V'
    
    def __isPulseData(self, token):
        return token[2] == 'P'
    
    def __extractData(self, token):
        # remove encoder identifier, id, data identifier
        sub_token = token[3:]
        # remove noise char if present
        sub_token = sub_token.replace(',','')
        return float(sub_token)
