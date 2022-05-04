#!/usr/bin/env python3
from cmath import pi
import threading
import rospy
import sys
from std_msgs.msg import Float64
import time
import numpy as np

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

Float64Msg = Float64
msg = """
Reading from the keyboard and Publishing to joint controller command!
---------------------------------------------------------------------
Moving end efector:
    Upper arrow:  advance in the direction of the positive y-axis (Y+)
    Lower arrow:  advance in the direction of the negative y-axis (Y-)
    Right arrow:  advance in the direction of the positive x-axis (X+)
    Left  arrow:  advance in the direction of the negative x-axis (X-)
Moving each joint:
    1: choose to move joint1
    2: choose to move joint2

s   : stop
0   : return to robot's starting position (HOME)
q/z : increase/decrease end efector speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'up':(0,1),
        'down':(0,-1,),
        'left':(-1,0),
        'right':(1,0),
        's':(0,0)
    }
speedBindings={
        'q':(1.1),
        'z':(.9)
    }
chooseJointBindings = {
        '1':"joint1",
        '2':"joint2"
}
moveJointBindings = {
        'w':(1),
        's':(-1),
        'x':(0)
    }
speedJointBindings={
        'd':(1.1),
        'a':(.9)
    }

class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.Joint1Publisher = rospy.Publisher('/robot2DoF/joint1_position_controller/command', Float64Msg, queue_size = 1)
        self.Joint2Publisher = rospy.Publisher('/robot2DoF/joint2_position_controller/command', Float64Msg, queue_size = 1)
        self.q1 = 0.0
        self.q2 = 0.0
        self.x = 0.0
        self.y = 0.0
        self.speed1 = 0.0
        self.speed2 = 0.0
        self.efSpeed = 0.0
        self.esc = 0.0
        self.key_pressed = False
        self.arrow_key = ""
        self.condition = threading.Condition()
        self.done = False
        self.home = 1
        self.moveEfector = True
        self.q1State = 0
        self.q2State = 0

        # Set timeout to None if rate is 0 (causes new_message to wait forever for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and (self.Joint1Publisher.get_num_connections() == 0 or self.Joint1Publisher.get_num_connections() == 0):
            if i == 4:
                print("Waiting for subscriber to connect to both\n{0} and\n{1}".format(self.Joint1Publisher.name,self.Joint2Publisher.name))
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, q1, q2, x, y, speed1, speed2, efSpeed, moveEfector):
        self.condition.acquire()
        self.q1 = q1
        self.q2 = q2
        self.speed1 = speed1/100
        self.speed2 = speed2/100
        self.efSpeed = efSpeed
        self.x = x
        self.y = y
        self.moveEfector = moveEfector
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0, self.moveEfector)
        self.join()

    def run(self):
        joint1_msg = Float64Msg()
        joint2_msg = Float64Msg()
        xState = 0.0
        yState = 0.0
        isPrinted=False
        alertMsg='A position unattainable by the robot is attempted to be reached.'
        while not self.done:
            self.condition.acquire()
            if self.moveEfector:
                (xState,yState)=joints2coor(self.q1State,self.q2State)
                (self.q1State,self.q2State)=coor2joints(xState-self.x*self.efSpeed,yState+self.y*self.efSpeed)

                if self.q2State<-0.8 or self.q2State>0.0:
                    (self.q1State,self.q2State)=coor2joints(xState,yState)
                    if not isPrinted:
                        print(alertMsg,end='\n\r')
                        isPrinted=True
                else:
                    isPrinted=False

            else:
                self.q1State = (self.q1State + self.q1*self.speed1)*self.home
                self.q2State = (self.q2State + self.q2*self.speed2)*self.home

                if self.q2State<-0.8:
                    self.q2State=-0.8
                    if not isPrinted:
                        print(alertMsg,end='\n\r')
                        isPrinted=True
                elif self.q2State>0.0:
                    self.q2State=0.0
                    if not isPrinted:
                        print(alertMsg,end='\n\r')
                        isPrinted=True
                else:
                    isPrinted=False

            joint1_msg.data = self.q1State
            joint2_msg.data = self.q2State

            self.condition.release()

            # Publish.
            self.Joint1Publisher.publish(joint1_msg)
            self.Joint2Publisher.publish(joint2_msg)

            time.sleep(0.001)

    def homePos(self):
        #return to robot's starting position (HOME)
        self.home=0
        self.update(0, 0, 0, 0, 0, 0, 0, False)
        time.sleep(0.0011)
        self.home=1
        
    def getArrowKey(self, key):
        arrow_key=""
        if self.key_pressed:
            self.key_pressed=False
            arrow_key=getArrowKey(key)
        if key=='\x1B' or self.esc>0:
            self.key_pressed =self.esc>0 and key=='\x5B'
            if self.key_pressed:
                self.esc=0
            else:
                self.esc+=1
        else:
            self.esc=0
        return arrow_key

def getJointMsg(nJoint):
    msgJoint = """
-------------------------------------------------------------
Moving joint{0}:
w/s : increase/decrease joint{0} position at a constant speed.
x   : stop

a/d : decrease/increase joint{0} speed by 10%
0   : stop and return to robot's starting position (HOME)
q   : stop and return to moving end efector
CTRL-C to quit
        """.format(nJoint)
    return msgJoint


def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def getArrowKey(key):
    arrow_key=""
    if key=="D":
        arrow_key="right"
    elif key=="A":
        arrow_key="up"
    elif key=="C":
        arrow_key="left"
    elif key=="B":
        arrow_key="down"
    return arrow_key

def coor2joints(x,y):
    q1 = np.arctan2(y,x)
    if q1<0:
        q1+=2*pi
    q2=np.linalg.norm([x,y])-0.8
    return [q1,q2]

def joints2coor(q1,q2):
    x = (q2+0.8) * np.cos(q1)
    y = (q2+0.8) * np.sin(q1)
    return [x,y]

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)

def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

def efVel(efSpeed):
    return "currently: final efector speed %.4f m/s" % (efSpeed)

def jointVel(speed,nJoint):
    vel=""
    if nJoint=='1':
        vel="rad/s"
    elif nJoint=='2':
        vel="m/s"
    return "currently: Joint%s speed %.4f %s" % (nJoint,speed,vel)

if __name__=="__main__":
    settings = saveTerminalSettings()

    rospy.init_node('teleop_robot2DoF_keyboard')
    speeds = {
        "speed1" : rospy.get_param("~speed1", 0.08),
        "speed2" : rospy.get_param("~speed2", 0.01)
    }
    efSpeed = rospy.get_param("~efSpeed", 0.0001)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_thread = PublishThread(repeat)

    q = {
        "q1" : 0,
        "q2" : 0
    }
    x  = 0
    y  = 0
    status = 0
    nJoint=''
    moveEfector=True
    elseCon=False

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(q["q1"], q["q2"], x, y, speeds["speed1"], speeds["speed2"],efSpeed,moveEfector)

        print(msg)
        print(efVel(efSpeed))
        while(1):
            key = getKey(settings)     
            arrowKey = pub_thread.getArrowKey(key)
            if arrowKey!="":
                key=arrowKey 

            if moveEfector:      
                if key in moveBindings.keys():
                    x = moveBindings[key][0]
                    y = moveBindings[key][1]
                elif key in speedBindings.keys():
                    efSpeed= efSpeed* speedBindings[key]
                    print(efVel(efSpeed))
                    if (status == 14):
                        print(msg)
                    status = (status + 1) % 15
                elif key in chooseJointBindings.keys():
                    moveEfector=False
                    pub_thread.update(0, 0, 0, 0, 0, 0, 0, moveEfector)
                    nJoint=key
                    print(getJointMsg(nJoint)) 
                    print(jointVel(speeds["speed{0}".format(nJoint)],nJoint))
                    status=0 
                else:
                    elseCon=True

            else:
                if key in moveJointBindings.keys(): 
                    q["q{0}".format(nJoint)] = moveJointBindings[key]
                elif key in speedJointBindings.keys():
                    speeds["speed{0}".format(nJoint)] = speeds["speed{0}".format(nJoint)]* speedJointBindings[key]
                    print(jointVel(speeds["speed{0}".format(nJoint)],nJoint))
                    if (status == 14):
                        print(getJointMsg(nJoint))
                    status = (status + 1) % 15
                elif key == '\x71': 
                    moveEfector=True
                    q = {
                        "q1" : 0,
                        "q2" : 0
                    }
                    x = 0
                    y = 0
                    print(msg)
                    print(efVel(efSpeed))
                    status=0
                else:
                    elseCon=True

            if elseCon:
                elseCon=False
                # return to home position if "0" pressed
                if key == '\x30':
                    pub_thread.homePos()
                    q = {
                        "q1" : 0,
                        "q2" : 0
                    }

                if (key == '\x03'):
                    break
 
            pub_thread.update(q["q1"], q["q2"], x, y, speeds["speed1"], speeds["speed2"],efSpeed,moveEfector)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)