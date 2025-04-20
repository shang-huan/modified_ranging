import matplotlib.pyplot as plt
import numpy as np
import re

THRESHOLD = 50

NOISE_THRESHOLD = 60

class uwbRangingData:
    T23 = []
    classicTof = []
    T3 = []
    
    d = []  #DSR 测距结果
    classicD = [] #DS-TWR 测距结果
    d3 = [] #T3测距结果
    
    origin = [] #原点
    path = [] #路径
    trueD = [] #真实距离
    
    def processConsoleLog(self, file):
        # [CalculateTof Finished]T23: 551,T3: 236,classicTof: 279,d: 129.258102,d3: 110.725631,classicD: 130.900207
        with open(file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                if "[CalculateTof Finished]" in line:
                    # print(line)
                    # [CalculateTof Finished][1] T23: 365,T3: -18395,classicTof: 272,d: 85.624694,d3: -8630.500000,classicD: 127.615982,trueD: 115.802291
                    # [CalculateTof Finished][0] T23: 748,T3: 19143,classicTof: 287,d: 175.471969,d3: 8981.443359,classicD: 134.653625,trueD: 0.0
                    match = re.search(r'\[CalculateTof Finished\]\[.*?\] T23: (-?\d+),T3: (-?\d+),classicTof: (-?\d+),d: (-?\d+.\d+),d3: (-?\d+.\d+),classicD: (-?\d+.\d+),trueD: (-?\d+.\d+)', line)
                    if match:
                        self.T23.append(int(match.group(1)))
                        self.T3.append(int(match.group(2)))
                        self.classicTof.append(int(match.group(3)))
                        self.d.append(float(match.group(4)))
                        self.d3.append(float(match.group(5)))
                        self.classicD.append(float(match.group(6)))
                        self.trueD.append(float(match.group(7)))
                       
        print("原始数据长度: ", len(self.d))

    def processOrigin(self, file):
        i = 0
        with open(file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                # 3-31-crazyflie,[0.01375999,1.16422343,0.00110769],0.00801940
                match = re.search(r'\[(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\]', line)
                if match:
                    x, y, z = match.groups()
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    i += 1
                    if(self.origin == []):
                        self.origin.append([x,y,z])
                    else:
                        self.origin[0][0] += x
                        self.origin[0][1] += y
                        self.origin[0][2] += z
                            
            self.origin[0][0] /= i
            self.origin[0][1] /= i
            self.origin[0][2] /= i
            self.origin[0][0] *= 100
            self.origin[0][1] *= 100
            self.origin[0][2] *= 100
            print("origin: ", self.origin)

    def processPath(self, file):
        with open(file, 'r') as f:
            lines = f.readlines()
            for line in lines:
                # 3-31-crazyflie,[0.01375999,1.16422343,0.00110769],0.00801940
                match = re.search(r'\[(-?\d+\.\d+),(-?\d+\.\d+),(-?\d+\.\d+)\]', line)
                if match:
                    x, y, z = match.groups()
                    x = float(x)
                    y = float(y)
                    z = float(z)
                    self.path.append([x,y,z])
        for i in range(len(self.path)):
            self.path[i][0] *= 100
            self.path[i][1] *= 100
            self.path[i][2] *= 100
        # print(self.path)
    
    def calTrueDistance(self):
        if(len(self.path) == 0):
            print("No path data")
            return
        if(len(self.origin) == 0):
            print("No origin data")
            return
        for i in range(len(self.path)):
            self.trueD.append(np.linalg.norm(np.array(self.path[i]) - np.array(self.origin[0])))
    
    def dataClean(self,left=0,right=-1,delNoise = False):
        if right == -1:
            right = len(self.d)
        self.d = self.d[left:right]
        self.classicD = self.classicD[left:right]
        self.trueD = self.trueD[left:right]
        
        if(delNoise):
            originLen = len(self.d)
            top = 0
            for i in range(len(self.d)):
                if abs(self.trueD[top] - self.d[top]) > NOISE_THRESHOLD:
                    self.trueD.pop(top)
                    self.d.pop(top)
                    self.classicD.pop(top)
                    top -= 1
                top += 1

            print("数据清洗长度变化:{}->{}".format(originLen, len(self.d)))
        
    def dataClean_justMove(self,low,high):
        originLen = len(self.d)
        top = 0
        for i in range(len(self.d)):
            if self.trueD[top] < low or self.trueD[top] > high:
                self.trueD.pop(top)
                self.d.pop(top)
                self.classicD.pop(top)
                top -= 1
            top += 1

        print("数据清洗长度变化:{}->{}".format(originLen, len(self.d)))
        
    def dataClean_justSide(self,low,high):
        originLen = len(self.d)
        top = 0
        for i in range(len(self.d)):
            if self.trueD[top] > low and self.trueD[top] < high:
                self.trueD.pop(top)
                self.d.pop(top)
                self.classicD.pop(top)
                top -= 1
            top += 1

        print("数据清洗长度变化:{}->{}".format(originLen, len(self.d)))
        
    def getViconMeanAndOffset(self,left=0,right=-1):
        if(right == -1):
            right = len(self.d)
        
        offset = 0
        length = 0
        
        if(left < 0 or right > len(self.d)):
            print("left or right out of range")
            return -1,-1
        
        for i in range(left,right):
            offset += (self.d[i]-self.trueD[i])
            length += 1
        
        mean = np.mean(self.trueD[left:right])
        offset = offset/length

        return mean,offset
     
    def dataAddViconOffset(self,offsetlow,meanlow,meanhigh,offsethigh):
        if meanlow == meanhigh:
            k = 0
        else:
            k = (offsethigh - offsetlow)/(meanhigh-meanlow)
        b = offsetlow - k*meanlow
            
        for i in range(1,len(self.trueD)):
            self.trueD[i] += (k*self.trueD[i] + b)
    
    def dataAlign_1(self):
        # 查找d数据波峰和波谷
        Down_d_index = []
        Up_d_index = []      
        for i in range(1,len(self.classicD) - 1):
            if self.classicD[i] > self.classicD[i + 1] and self.classicD[i] > self.classicD[i - 1]:
                Up_d_index.append(i)
            elif self.classicD[i] < self.classicD[i + 1] and self.classicD[i] < self.classicD[i - 1]:
                Down_d_index.append(i)
        print("Down_d_index: ", Down_d_index)
        print("Up_d_index: ", Up_d_index)
        
        
        # 剔除波峰波谷差距过低值
        index_Up_d = 0
        index_Down_d = 0
        while index_Up_d < len(Up_d_index) and index_Down_d < len(Down_d_index):
            if abs(self.classicD[Up_d_index[index_Up_d]] - self.classicD[Down_d_index[index_Down_d]]) < THRESHOLD:
                    Up_d_index.pop(index_Up_d)
                    Down_d_index.pop(index_Down_d)
            else:
                if(Up_d_index[index_Up_d] < Down_d_index[index_Down_d]):
                    index_Up_d += 1
                else:
                    index_Down_d += 1       
        
        # 查找trueD数据波峰和波谷
        Down_trueD_index = []
        Up_trueD_index = []
        for i in range(1,len(self.trueD) - 1):
            if self.trueD[i] > self.trueD[i + 1] and self.trueD[i] > self.trueD[i - 1]:
                Up_trueD_index.append(i)
            elif self.trueD[i] < self.trueD[i + 1] and self.trueD[i] < self.trueD[i - 1]:
                Down_trueD_index.append(i)
        # print("Down_trueD_index: ", Down_trueD_index)
        
        # 剔除波峰波谷差距过低值
        index_Up_trueD = 0
        index_Down_trueD = 0
        while index_Up_trueD < len(Up_trueD_index) and index_Down_trueD < len(Down_trueD_index):
            if abs(self.trueD[Up_trueD_index[index_Up_trueD]] - self.trueD[Down_trueD_index[index_Down_trueD]]) < THRESHOLD:
                    Up_trueD_index.pop(index_Up_trueD)
                    Down_trueD_index.pop(index_Down_trueD)
            else:
                if(Up_trueD_index[index_Up_trueD] < Down_trueD_index[index_Down_trueD]):
                    index_Up_trueD += 1
                else:
                    index_Down_trueD += 1
        
        print("d 波峰数：",len(Up_d_index))
        print("d 波谷数：",len(Down_d_index))
        print("trueD 波峰数：",len(Up_trueD_index))
        print("trueD 波谷数：",len(Down_trueD_index))

        #数据对准
        newD = []
        newClassicD = []
        newTrueD = []

        index_Up_d = 0
        index_Down_d = 0
        index_Up_trueD = 0
        index_Down_trueD = 0
        l_d = min(Up_d_index[index_Up_d], Down_d_index[index_Down_d])
        r_d = max(Up_d_index[index_Up_d], Down_d_index[index_Down_d])
        l_trueD = min(Up_trueD_index[index_Up_trueD], Down_trueD_index[index_Down_trueD])
        r_trueD = max(Up_trueD_index[index_Up_trueD], Down_trueD_index[index_Down_trueD])
        
        if (self.classicD[l_d]-self.classicD[r_d])*1.0/(self.trueD[l_trueD]-self.trueD[r_trueD]) < 0:
            print("波峰波谷不匹配")
            if(l_trueD == Up_trueD_index[index_Up_trueD]):
                l_trueD = r_trueD
                index_Up_trueD += 1
                r_trueD = Up_trueD_index[index_Up_trueD]
            else:
                l_trueD = r_trueD
                index_Down_trueD += 1
                r_trueD = Down_trueD_index[index_Down_trueD]
        
        newD.append(self.d[l_d])
        newClassicD.append(self.classicD[l_d])
        newTrueD.append(self.trueD[l_trueD])
        while index_Up_d < len(Up_d_index) and index_Down_d < len(Down_d_index) and index_Up_trueD < len(Up_trueD_index) and index_Down_trueD < len(Down_trueD_index):
            l_d = min(Up_d_index[index_Up_d], Down_d_index[index_Down_d])
            r_d = max(Up_d_index[index_Up_d], Down_d_index[index_Down_d])
            l_trueD = min(Up_trueD_index[index_Up_trueD], Down_trueD_index[index_Down_trueD])
            r_trueD = max(Up_trueD_index[index_Up_trueD], Down_trueD_index[index_Down_trueD])
            
            k = 1.0*(r_trueD-l_trueD)/(r_d-l_d)
            print("k:", k)
            index_d = l_d+1
            index_trueD = l_trueD+k
            index_trueD_int = round(index_trueD)
            # print("index_trueD:", index_trueD, index_trueD_int)
            while index_d <= r_d:
                newD.append(self.d[index_d])
                newClassicD.append(self.classicD[index_d])
                newTrueD.append(self.trueD[index_trueD_int])
                index_d += 1
                index_trueD += k
                index_trueD_int = round(index_trueD)
            if(l_d == Up_d_index[index_Up_d]):
                index_Up_d += 1
                index_Up_trueD += 1
            else:
                index_Down_d += 1
                index_Down_trueD += 1
        
        self.d = newD
        self.classicD = newClassicD
        self.trueD = newTrueD
        
    def dataAlign_2(self):
        # 查找d数据波峰和波谷
        Down_d_index = []
        Up_d_index = []      
        for i in range(1,len(self.classicD) - 1):
            if self.classicD[i] > self.classicD[i + 1] and self.classicD[i] > self.classicD[i - 1]:
                Up_d_index.append(i)
            elif self.classicD[i] < self.classicD[i + 1] and self.classicD[i] < self.classicD[i - 1]:
                Down_d_index.append(i)
        print("Down_d_index: ", Down_d_index)
        print("Up_d_index: ", Up_d_index)
        
        
        # 剔除波峰波谷差距过低值
        index_Up_d = 0
        index_Down_d = 0
        while index_Up_d < len(Up_d_index) and index_Down_d < len(Down_d_index):
            if abs(self.classicD[Up_d_index[index_Up_d]] - self.classicD[Down_d_index[index_Down_d]]) < THRESHOLD:
                    Up_d_index.pop(index_Up_d)
                    Down_d_index.pop(index_Down_d)
            else:
                if(Up_d_index[index_Up_d] < Down_d_index[index_Down_d]):
                    index_Up_d += 1
                else:
                    index_Down_d += 1       
        
        # 查找trueD数据波峰和波谷
        Down_trueD_index = []
        Up_trueD_index = []
        for i in range(1,len(self.trueD) - 1):
            if self.trueD[i] > self.trueD[i + 1] and self.trueD[i] > self.trueD[i - 1]:
                Up_trueD_index.append(i)
            elif self.trueD[i] < self.trueD[i + 1] and self.trueD[i] < self.trueD[i - 1]:
                Down_trueD_index.append(i)
        # print("Down_trueD_index: ", Down_trueD_index)
        
        # 剔除波峰波谷差距过低值
        index_Up_trueD = 0
        index_Down_trueD = 0
        while index_Up_trueD < len(Up_trueD_index) and index_Down_trueD < len(Down_trueD_index):
            if abs(self.trueD[Up_trueD_index[index_Up_trueD]] - self.trueD[Down_trueD_index[index_Down_trueD]]) < THRESHOLD:
                    Up_trueD_index.pop(index_Up_trueD)
                    Down_trueD_index.pop(index_Down_trueD)
            else:
                if(Up_trueD_index[index_Up_trueD] < Down_trueD_index[index_Down_trueD]):
                    index_Up_trueD += 1
                else:
                    index_Down_trueD += 1
        
        print("d 波峰数：",len(Up_d_index))
        print("d 波谷数：",len(Down_d_index))
        print("trueD 波峰数：",len(Up_trueD_index))
        print("trueD 波谷数：",len(Down_trueD_index))

        l_d = min(Up_d_index[0], Down_d_index[0])
        r_d = max(Up_d_index[-1], Down_d_index[-1])
        
        l_trueD = min(Up_trueD_index[0], Down_trueD_index[0])
        r_trueD = max(Up_trueD_index[-1], Down_trueD_index[-1])
        
        k = 1.0*(r_trueD-l_trueD)/(r_d-l_d)
        
        #数据对准
        newD = []
        newClassicD = []
        newTrueD = []
        
        index_d = l_d
        index_trueD = l_trueD
        index_trueD_int = round(index_trueD)
        while index_d <= r_d:
            newD.append(self.d[index_d])
            newClassicD.append(self.classicD[index_d])
            newTrueD.append(self.trueD[index_trueD_int])
            index_d += 1
            index_trueD += k
            index_trueD_int = round(index_trueD)
        
        self.d = newD
        self.classicD = newClassicD
        self.trueD = newTrueD
    
    def calErrorAndStd(self,left=0,right=-1):
        if(right == -1):
            right = len(self.d)
        
        error_D = []
        error_ClassicD = []
        
        if(left < 0 or right > len(self.d)):
            print("left or right out of range")
            return
        
        for i in range(left,right):
            error_D.append(abs(self.d[i] - self.trueD[i]))
            error_ClassicD.append(abs(self.classicD[i] - self.trueD[i]))
        
        # 计算平均误差和标准差  
        mean_error_D = np.mean(error_D)
        std_error_D = np.std(error_D)
        
        mean_error_ClassicD = np.mean(error_ClassicD)
        std_error_ClassicD = np.std(error_ClassicD)
        print("DSR平均误差: ", mean_error_D)
        print("DSR标准差: ", std_error_D)
        print("DS-TWR平均误差: ", mean_error_ClassicD)
        print("DS-TWR标准差: ", std_error_ClassicD)
        
    def calMeanandStd(self,left=0,right=-1):
        if(right == -1):
            right = len(self.d)
        
        if(left < 0 or right > len(self.d)):
            print("left or right out of range")
            return
        
        mean_d = np.mean(self.d[left:right])
        std_d = np.std(self.d[left:right])
        
        mean_classicD = np.mean(self.classicD[left:right])
        std_classicD = np.std(self.classicD[left:right])
        
        print("DSR平均值: ", mean_d)
        print("DSR标准差: ", std_d)
        print("DS-TWR平均值: ", mean_classicD)
        print("DS-TWR标准差: ", std_classicD)
    
    def showPeakAndTrough(self):
        dPeak = []
        dTrough = []
        classicDPeak = []
        classicDTrough = []
        viconPeak = []
        viconTrough = []
        
        changeThreshold = 120
        
        curMaxD = self.d[0]
        curMinD = self.d[0]
        curMaxClassicD = self.classicD[0]
        curMinClassicD = self.classicD[0]
        curMaxViconD = self.trueD[0]
        curMinViconD = self.trueD[0]
        
        isRisingD = False  
        isRisingClassicD = False
        isRisingViconD = False

        for i in range(1, len(self.d)):
            if self.d[i] > curMaxD:
                curMaxD = self.d[i]
            if self.d[i] < curMinD:
                curMinD = self.d[i]
            
            # 判断是否是从波峰开始下降，意味着波峰结束
            if isRisingD and (curMaxD - self.d[i]) > changeThreshold:
                dPeak.append(curMaxD)
                curMaxD = self.d[i]
                isRisingD = False
            
            # 判断是否是从波谷开始上升，意味着波谷结束
            elif not isRisingD and (self.d[i] - curMinD) > changeThreshold:
                dTrough.append(curMinD)
                curMinD = self.d[i]
                isRisingD = True
                
            if isRisingClassicD:
                if self.classicD[i] > curMaxClassicD:
                    curMaxClassicD = self.classicD[i]
                elif curMaxClassicD - self.classicD[i] > changeThreshold:
                    classicDPeak.append(curMaxClassicD)
                    print("classicDPeak:", curMaxClassicD)
                    curMinClassicD = self.classicD[i]
                    isRisingClassicD = False
            else:
                if self.classicD[i] < curMinClassicD:
                    curMinClassicD = self.classicD[i]
                elif self.classicD[i] - curMinClassicD > changeThreshold:
                    classicDTrough.append(curMinClassicD)
                    print("classicDTrough:", curMinClassicD)
                    curMaxClassicD = self.classicD[i]
                    isRisingClassicD = True
            
            if self.trueD[i] > curMaxViconD:
                curMaxViconD = self.trueD[i]
            if self.trueD[i] < curMinViconD:
                curMinViconD = self.trueD[i]
            # 判断是否是从波峰开始下降，意味着波峰结束
            if isRisingViconD and (curMaxViconD - self.trueD[i]) > changeThreshold:
                viconPeak.append(curMaxViconD)
                curMaxViconD = self.trueD[i]
                isRisingViconD = False
            # 判断是否是从波谷开始上升，意味着波谷结束
            elif not isRisingViconD and (self.trueD[i] - curMinViconD) > changeThreshold:
                viconTrough.append(curMinViconD)
                curMinViconD = self.trueD[i]
                isRisingViconD = True

        errorD = []
        errorClassicD = []
        for i in range(len(dPeak)):
            errorD.append(abs(dPeak[i] - viconPeak[i]))
            errorClassicD.append(abs(classicDPeak[i] - viconPeak[i]))
        for i in range(len(dTrough)):
            errorD.append(abs(dTrough[i] - viconTrough[i]))
            errorClassicD.append(abs(classicDTrough[i] - viconTrough[i]))
        # 计算平均误差和标准差
        mean_error_D = np.mean(errorD)
        std_error_D = np.std(errorD)
        mean_error_ClassicD = np.mean(errorClassicD)
        std_error_ClassicD = np.std(errorClassicD)
        print("DSR平均误差: ", mean_error_D)
        print("DSR标准差: ", std_error_D)
        print("DS-TWR平均误差: ", mean_error_ClassicD)
        print("DS-TWR标准差: ", std_error_ClassicD)
                
        plt.figure(figsize=(10, 6))
        plt.plot(dPeak, label='DSR Peak', color='blue', marker='*')
        plt.plot(dTrough, label='DSR Trough', color='blue', marker='o')
        plt.plot(classicDPeak, label='DS-TWR Peak', color='red', marker='*')
        plt.plot(classicDTrough, label='DS-TWR Trough', color='red', marker='o')
        plt.plot(viconPeak, label='Vicon Peak', color='green', marker='*')
        plt.plot(viconTrough, label='Vicon Trough', color='green', marker='o')
        
        plt.legend()
        plt.ylabel('Distance (cm)')
        plt.title('Peak and Trough of DSR, DS-TWR and Vicon')
        plt.show()
    
    def visualize(self,left=0,right=-1,ylimList = None):
        if(right == -1):
            right = len(self.d)
        
        if(left < 0 or right > len(self.d)):
            print("left or right out of range")
            return
        
        D = self.d[left:right]
        classicD = self.classicD[left:right]
        trueD = self.trueD[left:right]
        
        print(len(self.d), len(self.classicD), len(self.trueD))
        # 画图
        plt.figure(figsize=(10, 6))
        plt.plot(D, label='DSR', color='blue', marker='*')
        plt.plot(classicD, label='DS-TWR', color='red', marker='*')
        plt.plot(trueD, label='Vicon', color='green', marker='*')
        if(ylimList != None):
            plt.ylim(ylimList[0], ylimList[1])
        plt.xlabel('Time')
        plt.ylabel('Distance (cm)')
        plt.title('Distance Measurement')
        plt.legend()
        plt.grid()
        plt.show()

def showOrigin():
    originFile = "vicon_control/result/log.txt"
    data = uwbRangingData()
    data.processOrigin(originFile)

def showLog():
    consoleFile = "vicon_control/result/console_log.txt"
    data = uwbRangingData()
    data.processConsoleLog(consoleFile)
    data.calMeanandStd()
    data.visualize()

def showAftLog():
    date = "4-17"
    distance = "1"
    
    # consoleFile = "/Users/ou/Desktop/Vicon-UWB测距数据/4-17/静态实验/0.5/console_log.txt"
    # consoleFile = "/Users/ou/Desktop/Vicon-UWB测距数据/" + date + "/静态实验/" + distance + "/console_log.txt"
    # consoleFile = "/Users/ou/Desktop/Vicon-UWB测距数据/4-11/7"+ "/console_log.txt"
    
    consoleFile = "vicon_control/result/console_log.txt"

    data = uwbRangingData()
    data.processConsoleLog(consoleFile)
    data.dataClean(1493,1574)
    # lenTrueD = len(data.trueD)
    # data.trueD = data.trueD[0:lenTrueD-1]
    # data.d = data.d[1:lenTrueD]
    # data.classicD = data.classicD[1:lenTrueD]
    
    # meanlow,offsetlow = data.getViconMeanAndOffset(10,70)
    # meanhigh,offsethigh = data.getViconMeanAndOffset(160,210)
    # print("low offset: ", offsetlow)
    # print("high offset: ", offsethigh)
    # # data.dataAddViconOffset(offsetlow,meanlow,meanhigh,offsetlow)
    # data.dataAddViconOffset(offsetlow,meanlow,meanhigh,offsethigh)
    # data.dataClean_justMove(100, 160)
    # data.dataClean_justSide(100, 240)
    data.showPeakAndTrough()
    # data.calErrorAndStd()
    # data.calMeanandStd()
    # data.visualize(0,-1,[75,150])
    data.visualize()

if __name__ == "__main__":
    # showLog()
    showAftLog()
    # showOrigin()