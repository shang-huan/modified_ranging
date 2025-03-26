import re
import numpy as np
import matplotlib.pyplot as plt
import math


UWB_MAX_TIMESTAMP=1099511627776

class RangingTable:
    def __init__(self):
        self.sendBuffer = []
        self.receiveBuffer = []
        self.raArray = []
        self.dbArray = []
        self.rbArray = []
        self.daArray = []

    def test(self):
        # [Tx] localSeq:20,remoteSeq:20,TX:577226708440875009,RX:2306091676703072215
        # [Rx] localSeq:19,remoteSeq:2,TX:577226165438641665,RX:2306092170066908284
        msg = "[Tx]localSeq:19,remoteSeq:2,TX:577226165438641665,RX:2306092170066908284"
        pattern = "\[(Rx|Tx)\] *localSeq:(\d+),remoteSeq:(\d+),TX:(\d+),RX:(\d+)"
        match = re.match(pattern, msg)
        if match:
            print(match.groups())
        else:
            print("No match")

    def processDataFromFile(self,path):
        pattern = "\[(Rx|Tx)\] *localSeq:(\d+),remoteSeq:(\d+),TX:(\d+),RX:(\d+)"
        with open(path, "r") as file:
            for line in file:
                match = re.match(pattern, line)
                if match:
                    if match.groups()[0] == "Tx":
                        self.sendBuffer.append([int(match.groups()[1]), int(match.groups()[2]), int(match.groups()[3]), int(match.groups()[4])])
                    else:
                        self.receiveBuffer.append([int(match.groups()[1]), int(match.groups()[2]), int(match.groups()[3]), int(match.groups()[4])])
    
    def sortBuffer(self):
        self.sendBuffer = sorted(self.sendBuffer, key=lambda x: x[0])
        self.receiveBuffer = sorted(self.receiveBuffer, key=lambda x: x[0])
        
        # 去重
        i = 0
        while i < len(self.sendBuffer) - 1:
            if self.sendBuffer[i][0] == self.sendBuffer[i+1][0]:
                self.sendBuffer.pop(i)
            else:
                i += 1
                
        i = 0
        while i < len(self.receiveBuffer) - 1:
            if self.receiveBuffer[i][0] == self.receiveBuffer[i+1][0]:
                self.receiveBuffer.pop(i)
            else:
                i += 1
        
    def printBuffer(self):
        print("SendBuffer:")
        for i in self.sendBuffer:
            print(i)
        print("ReceiveBuffer:")
        for i in self.receiveBuffer:
            print(i)
            
    def doClassicRanging(self):
        tof = []
        for i in range(0, len(self.sendBuffer)-1):
            val = self.calClassicTof(self.receiveBuffer[i], self.sendBuffer[i], self.receiveBuffer[i+1])
            if(val[0] > 0):
                tof.append(val)
            val = self.calClassicTof(self.sendBuffer[i], self.receiveBuffer[i+1], self.sendBuffer[i+1])
            if(val[0] > 0):
                tof.append(val)
        return tof
    
    def calClassicTof(self, send1, receive, send2):
        Ra = (receive[3] - send1[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Rb = (send2[3] - receive[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Da = (send2[2] - receive[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Db = (receive[2] - send1[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        
        # print("Ra:",Ra,"Rb:",Rb,"Da:",Da,"Db:",Db)
        
        self.raArray.append(Ra)
        self.rbArray.append(Rb)
        self.daArray.append(Da)
        self.dbArray.append(Db)
        
        Tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
        d = Tof * 0.4691763978616
        
        print("ka:",(Ra)/(2*Tof+Db))
        
        return [Tof,d]

    # def doModifiedRanging(self):
    #     tof = []
    #     i = 0
        
    #     while i < len(self.sendBuffer)-2:
    #         if(i > 5):
    #             tof.append(self.calModifiedTof(self.receiveBuffer[i], self.sendBuffer[i], self.receiveBuffer[i+1],tof[-2][0],tof[-1][0]))
    #         else:
    #             tof.append(self.calClassicTof(self.receiveBuffer[i], self.sendBuffer[i], self.receiveBuffer[i+1]))
    #         i += 1
            
    #     return tof
        
    # def calModifiedTof(self, send1, receive, send2, tof1, tof2):
    #     Ra = (receive[3] - send1[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    #     Rb = (send2[3] - receive[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    #     Da = (send2[2] - receive[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    #     Db = (receive[2] - send1[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        
    #     T12 = tof1+tof2
        
    #     if(Rb/Db < 1):
    #         Tof = ((Ra*Rb - Da*Db) - (Rb)*T12)/(Db) - tof2
    #         print("Rb/Db",Rb/Db)
    #         if (Tof < 0):
    #             Tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
    #             print("Tof < 0, use classic")
    #     elif(Ra/Da < 1):
    #         Tof = ((Ra*Rb - Da*Db) - (Ra)*T12)/(Da) - tof2
    #         print("Ra/Da",Ra/Da)
    #         if(Tof < 0):
    #             Tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db) 
    #             print("Tof < 0, use classic")
    #     else:
    #         Tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db) 
    #         print("Classic")   
    #     d  = Tof * 0.4691763978616
    #     return [Tof,d]
    
    def doModifiedTofDoubleRanging(self):
        tofSum = []
        i = 0
        tofSum.append([0,0])
        
        while i < len(self.sendBuffer)-2:
            if(i >= 0 ):
                val = self.calModifiedTofDoubleRanging(self.receiveBuffer[i], self.sendBuffer[i], self.receiveBuffer[i+1], tofSum[-1][0])
                if(val[0] > 0):
                    tofSum.append(val)
                val = self.calModifiedTofDoubleRanging(self.sendBuffer[i], self.receiveBuffer[i+1], self.sendBuffer[i+1], tofSum[-1][0])
                if(val[0] > 0):
                    tofSum.append(val)
            else:
                val = self.calClassicTof(self.receiveBuffer[i], self.sendBuffer[i], self.receiveBuffer[i+1])
                if(val[0] > 0):
                    tofSum.append([val[0]*2,val[1]])
                val = self.calClassicTof(self.sendBuffer[i], self.receiveBuffer[i+1], self.sendBuffer[i+1])
                if(val[0] > 0):
                    tofSum.append([val[0]*2,val[1]])
            i += 1
        return tofSum
    
    def calModifiedTofDoubleRanging(self, send1, receive, send2, T12):
        Ra = (receive[3] - send1[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Rb = (send2[3] - receive[2] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Da = (send2[2] - receive[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        Db = (receive[2] - send1[3] + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
        
        if(Rb/Db > 1):
            T23 = ((Ra*Rb - Da*Db) - (Rb)*T12)/(Db)
            print("Rb/Db",Rb/Db)
            if (T23 < 0):
                T23 = 2*(Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
                print("T23 < 0, use classic")
        elif(Ra/Da > 1):
            T23 = ((Ra*Rb - Da*Db) - (Ra)*T12)/(Da)
            print("Ra/Da",Ra/Da)
            if(T23 < 0):
                T23 = 2*(Ra * Rb - Da * Db) / (Ra + Rb + Da + Db) 
                print("T23 < 0, use classic")
        else:
            T23 = 2*(Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
            print("Classic")
        d  = T23 * 0.4691763978616 / 2
        return [T23,d]
    
def reGetData(path):
    # [CalculateTof Finished]T23: 95,T3: 48,classicTof: 47,d: 22.285879,d3: 22.520467,classicD: 22.051290
    pattern = "\[CalculateTof Finished\]T23: (\d+),T3: (\d+),classicTof: (\d+),d: (\d+\.\d+),d3: (\d+\.\d+),classicD: (\d+\.\d+)"
    T23 = []
    classicTof = []
    d = []
    classicD = []
    d3 = []
    with open(path, "r") as file:
        for line in file:
            match = re.match(pattern, line)
            if match:
                print(match.groups())
                T23.append(int(match.groups()[0]))
                classicTof.append(int(match.groups()[2]))
                d.append(float(match.groups()[3]))
                classicD.append(float(match.groups()[5]))
                d3.append(float(match.groups()[4]))
    return T23,classicTof,d,classicD,d3
    
def funcMain1():
    rt = RangingTable()
    rt.processDataFromFile("/Users/ou/Desktop/飞行记录文件/测距/12-02/1-测距-1202-实验100cm，60cm，60-180cm.txt")
    rt.sortBuffer()
    # rt.plotDiff(rt.sendBuffer)
    
    tof = rt.doClassicRanging()
        
    # tof2 = rt.doModifiedRanging()
    # print("Modified:")
    # for i in tof2:
    #     print(i)
    
    plt.plot(range(0,len(tof)), [i[1] for i in tof], 'r' , label="Classic")
    # plt.plot(range(0,len(tof2)), [i[1] for i in tof2], label="Modified")

    # for i in range(0, len(rt.raArray)-1):
    #     print((rt.raArray[i]*rt.rbArray[i]-rt.daArray[i]*rt.dbArray[i],rt.raArray[i])/rt.raArray,rt.dbArray[i],(rt.dbArray[i+1]-rt.dbArray[i])/(rt.raArray[i+1]-rt.raArray[i]))

    T23 = rt.doModifiedTofDoubleRanging()
    # print("Modified Double:")
    # for i in T23:
    #     print(i)
    
    plt.plot(range(0,len(T23)), [i[1] for i in T23], label="Modified Double")
    
    true = 20
    d = [i[1] for i in tof]
    modifiedD = [i[1] for i in T23]
    avgD = np.mean(d)
    avgModifiedD = np.mean(modifiedD)
    stdD = np.std(d)
    stdModifiedD = np.std(modifiedD)
    print("Classic Avg:",avgD,"Std:",stdD)
    print("Modified Avg:",avgModifiedD,"Std:",stdModifiedD)
    
    # diff = []
    # for i in range(1,len(tof)):
    #     diff.append(T23[i][1] - tof[i][1])
    # plt.plot(range(0,len(diff)), diff, label="Diff")
    plt.legend()
    # plt.show()
    
def funcMain2():
    Ra = 61812033992
    Rb = 33980388989
    Da = 33980343352
    Db = 61812117346
    T12 = 74
    
    print((Ra * Rb - Da * Db))
    
    tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
    
    if(Ra/Da < 1):
        val1 = (Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db)
        #判断val1 有没有超过int64表达范围
        print(val1)
        if(val1 > 9223372036854775807 or val1 < -9223372036854775807):
            print("val1 > 9223372036854775807 or val1 < -9223372036854775807")
        tof2 = (((Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db))/2 - Ra*T12)/Da
    else:
        val1 = (Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db)
        print(val1)
        #判断val1 有没有超过int64表达范围
        if(val1 > 9223372036854775807 or val1 < -9223372036854775807):
            print("val1 > 9223372036854775807 or val1 < -9223372036854775807")
        tof2 = (((Ra + Da) * (Rb - Db) + (Ra - Da) * (Rb + Db))/2 - Rb*T12)/Db
    
    print(tof,tof2)
    
def funcMain3():
    [T23,classicTof,d,classicD,d3] = reGetData("/Users/ou/Desktop/飞行记录文件/测距/12-02/2-测距-1202-快速移动.txt")
    
    meanD = np.mean(d)
    meanClassicD = np.mean(classicD)
    stdD = np.std(d)
    stdClassicD = np.std(classicD)
    print("D Mean:",meanD,"Std:",stdD)
    print("Classic D Mean:",meanClassicD,"Std:",stdClassicD)
    
    T23_classicTof_d = []
    for i in range(0,len(T23)):
        T23_classicTof_d.append((T23[i] - classicTof[i])*0.4691763978616)
    
    # plt.plot(range(0,len(T23)), T23, 'r' , label="T23")
    # plt.plot(range(0,len(T23)), [i/2 for i in T23], 'r' , label="T23/2")
    # plt.plot(range(0,len(classicTof)), classicTof, label="Classic Tof")
    
    plt.plot(range(0,len(classicD)), classicD, label="Classic d")
    # plt.plot(range(0,len(d3)), d3, label="d3")
    plt.plot(range(0,len(T23_classicTof_d)), T23_classicTof_d, label="(T23 - Classic Tof) d")
    plt.plot(range(0,len(d)), d , label="d")
    plt.legend()
    plt.show()
    
    # diff = []
    # for i in range(0,len(T23)):
    #     diff.append(d[i] - classicD[i])
    # plt.plot(range(0,len(diff)), diff, label="Diff")
    
    # plt.legend()
    # plt.show()
    
def funcMain4():
    A1Tx = 577235089205522945
    A1Rx = 978145474944
    B2Tx = 577235759962583041
    B2Rx = 254043842077
    A3Tx = 577235057287014401
    A3Rx = 946227006474
    
    '''
        A3Tx            B2Rx A1Tx
            A3Rx B2Tx               A1Rx
    '''
    Ra = (B2Rx - A3Tx + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    Rb = (A1Rx - B2Tx + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    Da = (A1Tx - B2Rx + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    Db = (B2Tx - A3Rx + UWB_MAX_TIMESTAMP) % UWB_MAX_TIMESTAMP
    
    print(Ra,Db,Rb,Da)
    tof = (Ra * Rb - Da * Db) / (Ra + Rb + Da + Db)
    print(tof)

if __name__ == "__main__":
    # funcMain1()
    # funcMain2()
    funcMain3()
    # funcMain4()
    
    
    
    