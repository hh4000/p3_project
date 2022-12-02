import sounddevice as sd
from scipy.io.wavfile import write
from scipy import signal
import numpy as np
import librosa
from librosa import display
import matplotlib.pyplot as plt
from playsound import playsound
import cv2 as cv
import time
from time import sleep

#A duration for each sound sample along with a sampling rate
#This should be the same duration used when making the template
duration = 2
sampling_rate = 48000
d = 0.08 #meter distance between microphones
c = 343 #Speed of sound
#Choosing a file path for the noise template
file_path_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet sensitive (2 sek)' 
file_path_insensitive = r'C:\Users\silas\Desktop\3 microphones quiet insensitive (2 sek)'
file_path_very_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet very sensitive (2 sek)'
noise_template = np.loadtxt(file_path_sensitive)


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

class TriangularMicArr:
    def __init__(self,d):
        self.d = d
    def findSoundAngle(self,t12,t13,t23):
        a1 = np.arcsin(c*t23/self.d)
        if a1 >= 0:
            a2 = np.pi-a1
        else:
            a2 = -np.pi-a1
        a=[a1,a2]
        t12Expected = [self.d/c*np.cos(a1+np.pi/6),self.d/c*np.cos(a2+np.pi/6)]
        t13Expected = [self.d/c*np.cos(a1-np.pi/6),self.d/c*np.cos(a2-np.pi/6)] 
        if abs(t12-t12Expected[0]) < abs(t12-t12Expected[1]):
            t12ExpectedIndex=0
        else:
            t12ExpectedIndex=1
        if abs(t13-t13Expected[0]) < abs(t13-t13Expected[1]):
            t13ExpectedIndex=0
        else:
            t13ExpectedIndex=1
        if t12ExpectedIndex == t13ExpectedIndex:
            return a[t12ExpectedIndex]
        else:
            print("An error has occured:\n Angles not same\n")


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
        tri = TriangularMicArr(d)
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


