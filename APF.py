import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
import sys
import numpy as np
import message_filters
import math

class APF():
    def __init__(self):
    
        self.goal=np.array([int(x) for x in input("Podaj wartosc koordynatow x i y, gdzie ma isc robot\n").split()])
        #parametry robota
        print(self.goal)
        self.vmax=0.15
        self.KpOmega=0.6
        self.omega_max=0.2*np.pi
        # parametry apf
        self.position_accuracy=0.1
        #zeta to sila przyciagania
        self.zeta=1.1547
        # eta to sila odpychania
        self.eta=0.0732
        #dstar to odleglosc zmiany charakt przyciagania
        self.dstar=0.3
        #Qstar to odleglosc od ktorej robot widzi przeszkode
        self.Qstar=0.4
        self.error_theta_max=np.deg2rad(45)
        self.vref=0
        self.omega_ref=0
        #publish
        self.pub=rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        #zmienna publikowana
        self.vel=Twist()

        
    def callback(self,ODO,LAS):
        #przypisanie zmiennych
        self.range_min=LAS.range_min
        self.range_max=LAS.range_max
        self.angle_min=LAS.angle_min
        self.angle_max=LAS.angle_max
        self.Rtab=np.zeros(len(LAS.ranges))
        self.Rtab=np.asarray(LAS.ranges)
        self.angle_increment=LAS.angle_increment
        (self.roll, self.pitch, self.yaw) = euler_from_quaternion([
        ODO.pose.pose.orientation.x, ODO.pose.pose.orientation.y,
        ODO.pose.pose.orientation.z, ODO.pose.pose.orientation.w])
        
        self.position=np.array([ODO.pose.pose.position.x,ODO.pose.pose.position.y])
        self.position=self.position.astype(np.float)
       # liczenie i publikowanie
        #0
        self.computeAPF()
        self.vel.linear.x=self.vref
        self.vel.linear.y=0
        self.vel.linear.z=0

        self.vel.angular.x=0
        self.vel.angular.y=0
        self.vel.angular.z=self.omega_ref
        
        self.pub.publish(self.vel)
        
        
        
    def computeAPF(self):
        #liczenie predkosci
        
       
        #break
        if (np.linalg.norm(self.goal-self.position)>self.position_accuracy):

            #potencjal przyciagajacy
            if ((np.linalg.norm(self.position-self.goal))<=self.dstar):
                potAtt=self.zeta*(self.position-self.goal)
            else:
                potAtt=self.dstar/np.linalg.norm(self.position-self.goal) * self.zeta*(self.position-self.goal)
            
            #liczenie koordynatow xy wzgledem orientacji robota i potencjal odpychajacy
            obstxy=np.zeros((np.size(self.Rtab),2))
            potRep=np.array([0,0])
            for index,value in enumerate(self.Rtab):
                if np.isfinite(value):
                    obstxy[index,0]=value * np.cos(self.angle_min+self.angle_increment * index)
                    obstxy[index,1]=value * np.sin(self.angle_min+self.angle_increment * index)
                    if self.Rtab[index]<=self.Qstar:
                        potRep=potRep+ (self.eta*(1/self.Qstar - 1/self.Rtab[index])*(1/(self.Rtab[index])**2)) * (-(obstxy[index,:]))
                
            #kat=np.radians(90)
            #c,s=np.cos(kat), np.sin(kat)
            #MR = np.array([[c,-s],
                   #        [s,c]])
            
            #potvort=MR.dot(potRep)
            #potvort = MR@potRep
            
            potFinal=potRep+potAtt
            #potFinal=potvort+potAtt
            
            #referencyjnna wartosc predkosci i polozenia

            thetaref = np.arctan2(-potFinal[1],-potFinal[0])
            
            error_theta=thetaref - self.yaw
            if np.absolute(error_theta) <= self.error_theta_max:
                alpha = (self.error_theta_max - abs(error_theta)) / self.error_theta_max
                self.vref = min(alpha * np.linalg.norm(-potFinal), self.vmax)
            else:
                self.vref=0

            self.omega_ref = self.KpOmega * error_theta
            self.omega_ref = min(max(self.omega_ref,-self.omega_max),self.omega_max)
        else:
            self.vel.linear.x=0
            self.vel.angular.z=0
            self.pub.publish(self.vel)
            self.goal=np.array([int(x) for x in input("Podaj wartosc koordynatow x i y, gdzie ma isc robot\n").split()])
            
            
            
            
               

def APF_MAIN():
    
    apf=APF()

    rospy.init_node('APF_Cezaryv2', anonymous=False)

    Osub=message_filters.Subscriber('/odom', Odometry)
    Lsub=message_filters.Subscriber('/scan', LaserScan)

    ts=message_filters.ApproximateTimeSynchronizer([Osub,Lsub],queue_size=10, slop=0.1)
    ts.registerCallback(apf.callback)

    rospy.spin()
    
    
if __name__=='__main__':
    try:
        APF_MAIN()
    except rospy.ROSInterruptException:
        pass