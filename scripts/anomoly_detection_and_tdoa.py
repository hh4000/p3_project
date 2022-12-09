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
import rospy
from std_msgs.msg import Float32


### FIX PATHING ###
##Choosing a file path for the noise template
#file_path_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet sensitive (2 sek)' 
#file_path_insensitive = r'C:\Users\silas\Desktop\3 microphones quiet insensitive (2 sek)'
#file_path_very_sensitive = r'C:\Users\silas\Desktop\3 microphones quiet very sensitive (2 sek)'
#noise_template = np.loadtxt(file_path_sensitive)



class Triangular_mic_array:
    def __init__(self):
        self.d = 0.08 #m
        self.sampling_rate = 48000 # Hz
        self.speed_of_sound = 343 #m/s
        self.record_duration = 2 # s
        
        rospy.init_node('tdoa_angle_calculator',anonymous=True)
        self.angle_publisher = rospy.Publisher('/sound_identifier/goal_angle', Float32,queue_size=10)
        
        self.goal_angle = Float32()
        ### NO RATE DEFINED, AS IT MAY WORK WITHOUT ###
        #self.rate = rospy.Rate(0.3)
        
        
    def record(self):
        """Records using three microphones

        Returns:
            arr: three arrays containing the recordings of the three microphones
        """
        print("Recording")
        sd.default.device = 21
        my_recording = sd.rec(int(self.record_duration * self.sampling_rate), samplerate=self.sampling_rate, channels=8)
        sd.wait()
        print("done recording")
        #Storing the recordings from each device into their own variable
        my_recording1 = my_recording[:,0]
        my_recording2 = my_recording[:,1]
        my_recording3 = my_recording[:,2]
        return my_recording1, my_recording2, my_recording3
    
    def perform_fft(self,signal,samplerate):
        """performs a fast-fourier transform on a given signal

        Args:
            signal (arr): array symbolising the sound clip
            samplerate (int): samples

        Returns:
            ???: ???
        """
        #### PLEASE FIX THE VARIABLE NAMES ####
        ## I DO NOT UNDERSTAND WHAT THEY ARE ##
        fourier_transform = np.fft.fft(signal)
        n_samples = len(signal)
        amplitude = 2/n_samples * np.abs(fourier_transform)
        freq = np.fft.fftfreq(n_samples)*samplerate
        return n_samples, amplitude, freq, fourier_transform

    def anomoly_detection(self,signal,samplerate):
        """Checks if there is an anomoly in the sound clip

        Args:
            signal (array): recording of the signal
            samplerate (int): samples per second

        Returns:
            bool: whether or not there is an anomoly
        """
        #We set the bool "anomoly" as False to begin with and then perform an fft on the signal
        anomoly = False
        n_samples, amplitude, freq, ft = self.perform_fft(signal, samplerate)

        #We then do a for-loop that tests the newly recorded sound against the noise template
        for x in range(int(n_samples/2)):
            #If the sound has a frequency higher that the noise template it will be considered an anomoly
            if noise_template[x] < (amplitude[x]*100000):#Error because paths (see line 22-27)
                print("anomoly at freq: " + str(freq[x]) + " and amplitude: " + str(amplitude[x]*100000))
                return True
        #If not, then there is no anomoly
        return False
    def TDOA(self,signal1, signal2):
        """Find the Time difference of arrival of two signals

        Args:
            signal1 (arr): recording of one microphone
            signal2 (arr): recording of another microphone

        Returns:
            float: TDOA from signal 1 to signal 2
        """
        #We have numpy do a correlation on two signals
        correlation = signal.correlate(signal1, signal2, mode='same', method='auto')
        
        max_index = 0 
        u = 0 #u is the index of the peak in terms of the sampling rate
        #We then make a for-loop go through the correlation to find the index of the peak
        for x in range(len(correlation)): ## CAN PROBABLY BE CHANGED INTO enumerate ##
            if max_index <  correlation[x]:
                max_index = correlation[x]
                u = x
        #This index along with the sampling rate will give us the time difference between those two signals
        tdoa = (u-self.sampling_rate)/self.sampling_rate
        return tdoa
            
    def __find_angle_difference(self,ref_angle_rad, time_difference_seconds, time_id):
        """finds the angle difference compared to the reference angle given a certain time difference
            ### PRIVATE FUNCTION ###
        Args:
            ref_angle (float): angle expected to be close to the correct angle
            time_difference_seconds (float): time difference between sound wave arrivals (seconds)
            time_id (String): id of the time difference

        Returns:
            float: angle difference compared to reference angle given the time difference (radians)
        """
        # Check which time difference is worked on (changes factor)
        if time_id == 't12': factor = -1
        elif time_id=='t13': factor =  1
        else:
            #If not known id output error
            print()
            raise ValueError('Incorrect input for time_id; Expected \'t12\' or \'t13\', recieved ', time_id)
        # Two possible angles given the time differenc 
        angle_1 = factor*(np.pi/6-np.arccos(self.c/self.d*time_difference_seconds))
        angle_2 = factor*(np.arccos(self.c/self.d*time_difference_seconds)-np.pi*11/6)
        if abs(angle_2)> np.pi:
            # If angle 2 is out of bounds, use other method
            angle_2 = factor*(np.arccos(self.c/self.d*time_difference_seconds)+np.pi/6)
         
        ## Print statement for testing
        #print('\nTime difference: ',time_id,'\nAngle_1: ', angle_1,'\nAngle_2: ',angle_2)
        
        #Checks the angle distances to the reference angle
        angle_difference_1 = abs(angle_1-ref_angle_rad)
        if angle_difference_1 > np.pi:
            angle_difference_1 = 2*np.pi-angle_difference_1
            
        angle_difference_2 = abs(angle_2-ref_angle_rad)
        if angle_difference_2 > np.pi:
            angle_difference_2 = 2*np.pi-angle_difference_2
        #Gives angle closest to reference angle
        if angle_difference_1 < angle_difference_2:
            closest_angle_rad = angle_1
        else:
            closest_angle_rad = angle_2
            
        #Calculates difference between angle and reference considering direction.
        angle_difference = closest_angle_rad - ref_angle_rad
        if angle_difference < -np.pi:
            return angle_difference + 2*np.pi
        elif angle_difference > np.pi:
            return angle_difference - 2*np.pi
        else:
            return angle_difference
        
    def find_sound_angle(self,t12,t13,t23):
        """Find the directioned angle of the sound, given the time differences

        Args:
            t12 (float): TDOA of microphone 1 and 2
            t13 (float): TDOA of microphone 1 and 3
            t23 (float): TDOA of microphone 2 and 3

        Returns:
            float: angle of the sound in radians
        """
        #Calculate the 2 possible angles of the sound from the TDOA of mic 2 and 3
        a1 = np.arcsin(self.c*t23/self.d)
        if a1>=0:
            a2 =  np.pi - a1
        else:
            a2 = -np.pi - a1
        # puts angles into array
        a = [a1,a2]
        
        # Uses the TDOA of the other two sets of microphones to dedice the angle (a1,a2) that is most likely
        t12_expected = [self.d/self.c*np.cos(angle + np.pi/6) for angle in a]
        t13_expected = [self.d/self.c*np.cos(angle - np.pi/6) for angle in a]
        differences_1 = [abs(t12-time_difference) for time_difference in t12_expected]
        differences_2 = [abs(t13-time_difference) for time_difference in t13_expected]
        # If statements to decide correct angle
        if differences_1[0] < differences_1[1]:
            index_1 = 0
        else:
            index_1 = 1
        if differences_2[0] < differences_2[1]:
            index_2 = 0
        else:
            index_2 = 1
        
        # Error the case that the TDOA's are deemed impossible
        if index_1 != index_2:
            raise ValueError('Time differences are impossible')
        # Define angle_1 as the correct angle
        angle_1 = a[index_1]
        # calculates the angles given the other sets of TDOA and gives the difference
        angle_difference_2 = self.__find_angle_difference(angle_1,t12,'t12')
        angle_difference_3 = self.__find_angle_difference(angle_1,t13,'t13')
        
        #print('\nAngle_1:\n',angle_1,'\nAngle differences:\n',angle_difference_2,'\n',angle_difference_3)
        # Finds the average angle using differences
        average_angle = angle_1 + (angle_difference_2+angle_difference_3)/3
        
        # returns the average angles, including statements for overflow (num > pi) an underflow (num <= pi)
        if average_angle > np.pi:
            return average_angle - 2*np.pi
        if average_angle <= -np.pi:
            return average_angle + 2*np.pi
        return average_angle
    def run(self):
        while not rospy.is_shutdown():
            #If all three recordings show anomoly, then we perform the TDOA
            rec_1, rec_2, rec_3  = self.record()
            if self.anomoly_detection(rec_1,self.sampling_rate) == True and self.anomoly_detection(rec_2,self.sampling_rate) == True and self.anomoly_detection(rec_3,self.sampling_rate) == True:
                print("Anomoly detected")
                t12 = self.TDOA(rec_1,rec_2)
                t23 = self.TDOA(rec_2,rec_3)
                t13 = self.TDOA(rec_1,rec_3)
                self.goal_angle.data = self.findSoundAngle(t12,t13,t23)
                #print("rad: " + str(rad))
                #angle = rad*(180/np.pi)
                #print("anomoly")
                #print("We have found an anomoly at angle: " + str(angle))
                self.angle_publisher.publish(self.goal_angle)
                rospy.loginfo('Anomoly found at angle: %f', self.goal_angle.data)
            #motionDetectionNMS(0)
            #sd.wait()
            
            # and move the sd.wait() into the recording function again
            #If we are just gonna run the montion detection by itself, then we can just remove the motion function in here 

if __name__ == '__main__':
    try:
        microphone_array = Triangular_mic_array()
        microphone_array.run()
    except rospy.ROSInterruptException:
        pass


#####################################
### Below is for testing purposes ###
#####################################
#i = 0
#reps = 100
##Replace with "True" when we do it indefinitely 
#no_anomolies = 0
#anomolies = 0
#
#
#while i<reps:
#    rec1, rec2, rec3 = recording_devices()
#    #If all three recordings show anomoly, then we perform the TDOA
#    if anomoly_detection(rec1,sampling_rate) == True and anomoly_detection(rec2,sampling_rate) == True and anomoly_detection(rec3,sampling_rate) == True:
#        print("Anomoly detected")
#        t12 = TDOA(rec1,rec2)
#        t23 = TDOA(rec2,rec3)
#        t13 = TDOA(rec1,rec3)
#        tri = Triangular_mic_array(d)
#        #rad = tri.findSoundAngle(t12,t13,t23)
#        #print("rad: " + str(rad))
#        #angle = rad*(180/np.pi)
#        #print("anomoly")
#        #print("We have found an anomoly at angle: " + str(angle))
#        anomolies = anomolies+1
#    else:
#        print("No anomoly detected")
#        no_anomolies = no_anomolies+1
#    i = i+1
#    print(i)
#    #motionDetectionNMS(0)
#    #sd.wait()
#
#
#print("It found anomolies " + str(anomolies) + " times")
#print("It found no anomolies " + str(no_anomolies)+ " times")
#
##If we are just gonna run the montion detection by itself, then we can just remove the motion function in here 
## and move the sd.wait() into the recording function again