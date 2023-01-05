#!/usr/bin/env python3
import numpy as np
from sqlalchemy import true
import rospy
from std_msgs.msg import Int8MultiArray, MultiArrayDimension, Float32MultiArray
import time
from functools import partial
from bocd import *
from types import ModuleType, FunctionType

one_minute = 1e+9*60
one_second = 1e+9 #in nanao second

#Bayes CPD
LAMBDA = 100
ALPHA = .1
BETA = 1.
KAPPA = 1.
MU = 0.
DELAY = 7
THRESHOLD = 0.2
cp_per_min = 3 

max_time = 60 #How long you have to hold still (in sec)
max_queue = 10 #Max elemts in queue
max_elements = 900 #How many elemts to save
ignore_wrist = true

BLACKLIST = type, ModuleType, FunctionType

start_time = time.time_ns()
fft_threashold = 0.1 #Ab wann frequenzen akzeptiert werden

class Musclework():
    def __init__(self):
        #Paramter
        self.rula_score = []
        
        self.bs = [] #Array of the CPD
        self.timestamps = []
        self.last_move_body_part = [0,0] #when was the last change point

        self.angles = []
        self.queue = []
        self.change_points = [] #When was the last changepoint for the angle
        self.angle_names = [
            "upper_arm_left",
            "upper_arm_right",
            "lower_arm_left",
            "lower_arm_right",
            "neck",
            "trunk",
            "legs",
            "wrist_left",
            "wrist_right"]
        
        #FFT
        self.fft = []

        #Adds data:
        if ignore_wrist:
            tmp_max = 7
        else:
            tmp_max = 9

        for i in range(tmp_max):
            cpd = BOCD(partial(constant_hazard, LAMBDA),
                StudentT(ALPHA, BETA, KAPPA, MU))
            self.bs.append(cpd)
            self.angles.append([])
            self.queue.append([])
            self.change_points.append([])
            self.fft.append(-1.0)
            
        #Pub sub
        rospy.Subscriber("/ergonomics/joint_angles", Float32MultiArray, self.callback)
        rospy.Subscriber('/ergonomics/rula', Float32MultiArray, self.score_callback)

        self.pub = rospy.Publisher('/musclework/musclework', Int8MultiArray, queue_size=10)
        

    def score_callback(self,data):
        if ignore_wrist:
            data.data = data.data[0:7]
        for i in range(len(data.data)):
            self.rula_score.append(data.data[i])
        self.rula_score = self.rula_score[-max_elements:]

    #Hier kommen die daten rein
    def callback(self,data):
        #Die letzten beiden Elemente werden rausgeworfen
        if ignore_wrist:
            data.data = data.data[0:7]
        
        #rospy.loginfo(data.data)

        #Neues wird hinzu gefügt und altes rausgeschmissen
        data.data = list(data.data)

        for i in range(len(data.data)):
            self.queue[i].append(data.data[i])
            self.queue[i] = self.queue[i][-max_queue:]

        self.timestamps.append(time.time_ns()-start_time)
        self.timestamps = self.timestamps[-max_elements:]
            

    #Hier werden die daten verarbeitet
    def process(self):
        #Data handle here:
        if len(self.queue[0]) > 0:
            for i in range(len(self.queue)):
                #Add data
                for data_point in self.queue[i]:
                    self.angles[i].append(data_point)
                self.angles[i] = self.angles[i][-max_elements:]
                
                #alte Changepoints Entfernen
                for cp in self.change_points[i]:
                    if self.timestamps[-1] - cp > one_minute:
                        self.change_points[i].remove(cp)

                #Bayes complete
                tmp_cp = self.BOCD_prune(self.bs[i], self.queue[i])
                if tmp_cp != []:
                    self.change_points[i].append(self.timestamps[-1]) #hier wird die zeit des letzten change points genommen
                
                #Aufräumen
                self.queue[i] = []
                
                #FFT
                if len(self.angles[i]) > max_elements/4:
                    self.fft[i] = self.fastFourierTransform(self.angles[i])
                else:
                    self.fft[i] = -1.0
            
            #Plus punkt vergeben
            #[Arme, Körper]
            self.last_move_body_part = [0,0] #Wir gehen davon aus der Körper ist dynamisch
            self.repetive_body_part = [0,0]
            tmp_time = (time.time_ns()-start_time)
            if tmp_time >= one_minute:
                for i in range(4):
                    #Wenn mehr als max_time vergangen ist
                    if len(self.change_points[i]) < cp_per_min:
                        if self.rula_score[i] > 1:
                            self.last_move_body_part[0] = 1
                    #FFT
                    if self.fft[i] > fft_threashold: #Hier gerne auch mehr testen
                        self.repetive_body_part[0] = 1

                for i in range(4,7):
                    #Neck, trunk, legs
                    if len(self.change_points[i]) < cp_per_min:
                        if self.rula_score[i] > 1:
                            self.last_move_body_part[1] = 1
                    #FFT
                    if self.fft[i] > fft_threashold: #Hier gerne auch mehr testen
                        self.repetive_body_part[1] = 1


            #Zusammen rechnen
            self.end_score = [0,0]
            self.end_score[0] = min(1, self.last_move_body_part[0]+ self.repetive_body_part[0])
            self.end_score[1] = min(1, self.last_move_body_part[1]+ self.repetive_body_part[1])

            #Publish
            print("Statisch: " + str(self.last_move_body_part[0]) + ", " + str(self.last_move_body_part[1]) + " | Repetetiv: " + str(self.repetive_body_part[0]) + ", " + str(self.repetive_body_part[1]))
            self.pub.publish(createInt8MultiArray(self.end_score))
            
    def fastFourierTransform(self,data):

        #settings
        use_window = False
        method = "peak"

        if use_window:
            window = np.hanning(len(data))
            data = data * window

        #Mitlere Frequenz?
        dt = (len(self.timestamps))/(self.timestamps[-1]-self.timestamps[0])*one_second
        n = len(data)
        fhat = np.fft.fft(data,n)
        PSD = fhat * np.conj(fhat) / n
        indices = PSD > fft_threashold
        fhat = indices * fhat
        
        abs_PSD = []
        #PSD
        if use_window:
            abs_PSD = np.abs(PSD[1:len(PSD)//2])
        else:
            abs_PSD = np.abs(PSD[1:len(PSD)//2])


        freq = -1
        if method == "weighted_avg":
            #weighted Avg
            weight = 0
            avg = 0
            freq = 0
            
            for i in range(len(abs_PSD)):
                weight += abs_PSD[i]
                if use_window:
                    avg = (dt * (i+1)/n) *abs_PSD[i]
                else:
                    avg = (dt * (i+1)/n) *abs_PSD[i]
            freq = avg/weight
            
        elif method == "peak":
            #find Peak
            argmax = np.argmax(abs_PSD)
            if use_window:
                freq = (dt * (argmax+1)/n)
            else:
                freq = (dt * (argmax+1)/n)
        

        elif method == "double_peak":
            #find Peaks
            tuples = sorted(((value, index) for index, value in enumerate(abs_PSD)), reverse=True)
            index = []
            index.append(tuples[0][1])
            index.append(tuples[1][1])


            if use_window:
                freq = ((dt * (index[0]+1)/n) + (dt * (index[1]+1)/n)) /2
            else:
                freq = ((dt * (index[0]+1)/n) + (dt * (index[1]+1)/n)) /2
        return freq

    
    def BOCD_prune(self,bocd, data):
        """Tests that changepoints are detected while pruning.
        """
        changepoints = []
        if bocd.t <= DELAY:
            for x in data:
                bocd.update(x)
        else:
            for x in data:
                bocd.update(x)
                if bocd.growth_probs[DELAY] >= THRESHOLD:
                    changepoints.append(bocd.t - DELAY + 1)
                    bocd.prune(bocd.t-DELAY)
        if bocd.t - bocd.t0 > max_elements:
            bocd.prune(bocd.t-DELAY)
        return changepoints


def createMultiArray(data):
    msg = Float32MultiArray()
    dim = MultiArrayDimension()
    dim.label = "RULA Scores"
    dim.stride = 1
    dim.size = len(data)
    msg.layout.dim.append(dim)
    msg.data = data
    return msg  

def createInt8MultiArray(data):
    msg = Int8MultiArray()
    dim = MultiArrayDimension()
    dim.label = "RULA Scores"
    dim.stride = 1
    dim.size = len(data)
    msg.layout.dim.append(dim)
    msg.data = data
    return msg

def musclework_start():
    print("Setup Musclework")

    rospy.init_node('Musclework', anonymous=True)
    musclework = Musclework()
    rate = rospy.Rate(2)
    
    while not rospy.is_shutdown():
        try:
            musclework.process()
            rate.sleep()
        except KeyboardInterrupt:
            print("Shutting down")


if __name__ == '__main__':
    try:
        musclework_start()
    except rospy.ROSInterruptException:
        pass