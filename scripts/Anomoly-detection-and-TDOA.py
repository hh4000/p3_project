#Import libraries
import sounddevice as sd
#from scipy.io.wavfile import write
from scipy import signal
import numpy as np
#import librosa
#from librosa import display
#import matplotlib.pyplot as plt
#from playsound import playsound
#import cv2 as cv
#import time
#from time import sleep

#A duration for each sound sample along with a sampling rate
#This should be the same duration used when making the template
duration = 2
sampling_rate = 48000
d = 0.08 #meter distance between microphones
c = 343 #Speed of sound


### FIX PATHING ###
##Choosing a file path for the noise template
#file_path_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet sensitive (2 sek)' 
#file_path_insensitive = r'C:\Users\silas\Desktop\3 microphones quiet insensitive (2 sek)'
#file_path_very_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet very sensitive (2 sek)'
#noise_template = np.loadtxt(file_path_sensitive)


#Recording from all devices
def recording_devices():
    print("Recording")
    sd.default.device = 21
    myRecording = sd.rec(int(duration *sampling_rate), samplerate=sampling_rate, channels=8)
    sd.wait()
    print("done recording")
    #Storing the recordings from each device into their own variable
    myRecording1 = myRecording[:,0]
    myRecording2 = myRecording[:,1]
    myRecording3 = myRecording[:,2]
    return myRecording1, myRecording2, myRecording3

#Function to perform an fft on any signal to find freq, amp, samples
def perform_fft(signal,sr):
    ft = np.fft.fft(signal)
    n_samples = len(signal)
    amplitude = 2/n_samples * np.abs(ft)
    freq = np.fft.fftfreq(n_samples)*sr
    return n_samples, amplitude, freq, ft

#Function for detecting anomolies in signals
def anomoly_detection(signal,sr):
    #We set the bool "anomoly" as False to begin with and then perform an fft on the signal
    anomoly = False
    n_samples, amplitude, freq, ft = perform_fft(signal, sr)

    #We then do a for-loop that tests the newly recorded sound against the noise template
    for x in range(int(n_samples/2)):
        #If the sound has a frequency higher that the noise template it will be considered an anomoly
        if noise_template[x] < (amplitude[x]*100000):
            print("anomoly at freq: " + str(freq[x]) + " and amplitude: " + str(amplitude[x]*100000))
            anomoly = True
            return True
    #If not, then there is no anomoly
    if anomoly == False:
        return False

def TDOA(signal1, signal2):
    #We have numpy do a correlation on two signals
    corr = signal.correlate(signal1, signal2, mode='same', method='auto')
    
    max3 = 0
    u = 0 #u is the index of the peak in terms of the sampling rate
    #We then make a for-loop go through the correlation to find the index of the peak
    for x in range(len(corr)):
        if max3 < corr[x]:
            max3 = corr[x]
            u = x
    #This index along with the sampling rate will give us the time difference between those two signals
    tdoa = (u-sampling_rate)/sampling_rate
    return tdoa

class Triangular_mic_array:
    def __init__(self,d):
        self.d = d
    
    def __find_angle_difference(self,ref_angle_rad, time_difference_seconds, time_id):
        """finds the angle closest to the reference angle
            ### PRIVATE FUNCTION ###
        Args:
            ref_angle (float): angle expected to be close to the correct angle
            time_difference_seconds (float): time difference between sound wave arrivals (seconds)
            time_id (String): id of the time difference

        Returns:
            float: angle given the time difference (radians)
        """
        if time_id == 't12': factor = -1
        elif time_id=='t13': factor =  1
        else:
            print('Incorrect input for time_id; Expected \'t12\' or \'t13\', recieved ', time_id)
            return 'Error'
        
        angle_1 = factor*(np.pi/6-np.arccos(c/self.d*time_difference_seconds))
        angle_2 = factor*(np.arccos(c/self.d*time_difference_seconds)-np.pi*11/6)
        if abs(angle_2)> np.pi:
            angle_2 = factor*(np.arccos(c/self.d*time_difference_seconds)+np.pi/6)
         
        ## Print statement for testing
        #print('\nTime difference: ',time_id,'\nAngle_1: ', angle_1,'\nAngle_2: ',angle_2)
        
        angle_difference_1 = abs(angle_1-ref_angle_rad)
        if angle_difference_1 > np.pi:
            angle_difference_1 = 2*np.pi-angle_difference_1
            
        angle_difference_2 = abs(angle_2-ref_angle_rad)
        if angle_difference_2 > np.pi:
            angle_difference_2 = 2*np.pi-angle_difference_2
        
        if angle_difference_1 < angle_difference_2:
            return_angle_rad = angle_1
        else:
            return_angle_rad = angle_2
        #if return_angle_rad < 0:
        #    return return_angle_rad + 2*np.pi - ref_angle_rad
        #else:
        angle_difference = return_angle_rad - ref_angle_rad
        if angle_difference < -np.pi:
            return angle_difference + 2*np.pi
        elif angle_difference > np.pi:
            return angle_difference - 2*np.pi
        else:
            return angle_difference
        
    def find_sound_angle(self,t12,t13,t23):
        a1 = np.arcsin(c*t23/self.d)
        if a1>=0:
            a2 =  np.pi - a1
        else:
            a2 = -np.pi - a1
        a = [a1,a2]
        t12_expected = [self.d/c*np.cos(angle + np.pi/6) for angle in a]
        t13_expected = [self.d/c*np.cos(angle - np.pi/6) for angle in a]
        
        differences_1 = [abs(t12-time_difference) for time_difference in t12_expected]
        differences_2 = [abs(t13-time_difference) for time_difference in t13_expected]
        
        if differences_1[0] < differences_1[1]:
            index_1 = 0
        else:
            index_1 = 1
        if differences_2[0] < differences_2[1]:
            index_2 = 0
        else:
            index_2 = 1
            
        if index_1 != index_2:
            print('\nTime difference error! Time differences impossible.\n')
            return 'error'
        angle_1 = a[index_1]
        
        angle_difference_2 = self.__find_angle_difference(angle_1,t12,'t12')
        angle_difference_3 = self.__find_angle_difference(angle_1,t13,'t13')
        
        #print('\nAngle_1:\n',angle_1,'\nAngle differences:\n',angle_difference_2,'\n',angle_difference_3)
        
        average_angle = angle_1 + (angle_difference_2+angle_difference_3)/3
        
        if average_angle > np.pi:
            return average_angle - 2*np.pi
        if average_angle < -np.pi:
            return average_angle + 2*np.pi
        return average_angle


i = 0
reps = 100
#Replace with "True" when we do it indefinitely 
no_anomolies = 0
anomolies = 0


while i<reps:
    rec1, rec2, rec3 = recording_devices()
    #If all three recordings show anomoly, then we perform the TDOA
    if anomoly_detection(rec1,sampling_rate) == True and anomoly_detection(rec2,sampling_rate) == True and anomoly_detection(rec3,sampling_rate) == True:
        print("Anomoly detected")
        t12 = TDOA(rec1,rec2)
        t23 = TDOA(rec2,rec3)
        t13 = TDOA(rec1,rec3)
        tri = Triangular_mic_array(d)
        #rad = tri.findSoundAngle(t12,t13,t23)
        #print("rad: " + str(rad))
        #angle = rad*(180/np.pi)
        #print("anomoly")
        #print("We have found an anomoly at angle: " + str(angle))
        anomolies = anomolies+1
    else:
        print("No anomoly detected")
        no_anomolies = no_anomolies+1
    i = i+1
    print(i)
    #motionDetectionNMS(0)
    #sd.wait()


print("It found anomolies " + str(anomolies) + " times")
print("It found no anomolies " + str(no_anomolies)+ " times")

#If we are just gonna run the montion detection by itself, then we can just remove the motion function in here 
# and move the sd.wait() into the recording function again


