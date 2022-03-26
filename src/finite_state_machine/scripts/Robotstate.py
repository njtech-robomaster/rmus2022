#!/usr/bin/env python3
from threading import local
from Robot import Robot
from transitions.extensions import GraphMachine
import transitions
import rospy
import eventlet
import json


class Robotstates(Robot):
    # Define states

    statesList = [transitions.State(name='init', on_enter=['init']),
                  transitions.State(name='Asleep', on_enter=['Asleep']),
                  transitions.State(name='Observe', on_enter=['Observe'],on_exit=['exitObserve']),
                  transitions.State(name='FindingOre',
                                    on_enter=['FindingOre']),
                  transitions.State(name='CatchOre', on_enter=['CatchOre']),
                  transitions.State(name='TransportingOre',
                                    on_enter=['TransportingOre']),
                  transitions.State(name='PlaneOre', on_enter=['PlaneOre']),
                  transitions.State(name='Back', on_enter='Back')]

    transitionsList = [{'trigger': 'toAsleep', 'source': 'init', 'dest': 'Asleep'},
                       {'trigger': 'toObserve', 'source': 'Asleep',
                        'dest': 'Observe', 'conditions': 'isStart'},
                       {'trigger': 'toObserve', 'source': 'FindingOre',
                        'dest': 'Observe'},
                       {'trigger': 'toObserve', 'source': 'Observe',
                        'dest': 'Observe'},
                       {'trigger': 'toFindingOre', 'source': 'Observe',
                        'dest': 'FindingOre', 'conditions': 'isGetNum'},
                       {'trigger': 'toCatchOre', 'source': 'FindingOre',
                        'dest': 'CatchOre', 'conditions': 'isReach'},
                       {'trigger': 'toTransportingOre', 'source': 'CatchOre',
                        'dest': 'TransportingOre', 'conditions': 'isCatch'},
                       {'trigger': 'toPlaneOre', 'source': 'TransportingOre',
                        'dest': 'PlaneOre', 'conditions': 'isReach'},
                       {'trigger': 'toFindingOre', 'source': 'PlaneOre',
                        'dest': 'FindingOre', 'conditions': 'isPlane'},
                       {'trigger': 'isFinish', 'source': 'PlaneOre',
                        'dest': 'Back'},
                       {'trigger': 'Clean_up', 'source': 'Back', 'dest': 'init'}]

    def __init__(self) -> None:
        super(Robotstates, self).__init__()

        rospy.init_node('State_machine', anonymous=True)

        self.machine = transitions.Machine(
            model=self, states=Robotstates.statesList, transitions=Robotstates.transitionsList, initial='init')
        

    def getGraph(self):
        return GraphMachine(model=self, states=self.statesList, transitions=self.transitionsList, initial='init')

    def init(self):
        rospy.loginfo("State : Init")
        self.toAsleep()

    def Asleep(self):
        self.isStart = True
        rospy.loginfo("State : Asleep")
        self.toObserve()

    def Observe(self):

        rospy.loginfo("State : Observe")
        self.statePub.publish("Observe")
        self.onObserve=True

        
        rospy.loginfo("Move to the first location...")
        self.move(0.250, 1.541, 0.000, 0.000, 0.068, 0.998)
        while eventlet.Timeout(5, False):
            if self.isGetNum and self.arucoNum == 3:
                self.toFindingOre()
                break


        rospy.loginfo("Move to the second location...")
        self.move(0.250, 1.541, 0.000, 0.000, 0.068, 0.998)
        while eventlet.Timeout(5, False):
            if self.isGetNum and self.arucoNum == 3:
                self.toFindingOre()
                break


        rospy.loginfo("Move to the third location...")
        self.move(0.250, 1.541, 0.000, 0.000, 0.068, 0.998)
        while eventlet.Timeout(5, False):
            if self.isGetNum and self.arucoNum == 3:
                self.toFindingOre()
                break
            elif self.isGetNum:
                self.toFindingOre()
                break


        self.toObserve()

    def FindingOre(self):
        self.statePub.publish("FindingOre")
        rospy.loginfo("State : FindingOre")


        if self.findoreCount > len(self.aruco):
            rospy.logwarn("The number of ore searches exceeded the number of ore information obtained, Robot will return!")
            self.toBack()

        if self.aruco[self.findoreCount] == 1:
            rospy.loginfo("Move to No. 1 Mine")
            self.move()
            self.findoreCount+=1
            self.toCatchOre()
        elif self.aruco[self.findoreCount] == 2:
            rospy.loginfo("Move to No. 2 Mine")
            self.move()
            self.findoreCount+=1
            self.toCatchOre()
        elif self.aruco[self.findoreCount] == 3:
            rospy.loginfo("Move to No. 3 Mine")
            self.move()
            self.findoreCount+=1
            self.toCatchOre()
        elif self.aruco[self.findoreCount] == 4:
            rospy.loginfo("Move to No. 4 Mine")
            self.move()
            self.findoreCount+=1
            self.toCatchOre()
        elif self.aruco[self.findoreCount] == 5:
            rospy.loginfo("Move to No. 5 Mine")
            self.move()
            self.findoreCount+=1
            self.toCatchOre()
        elif self.aruco[self.findoreCount] == 0:
            rospy.loginfo("Move to No. 5 Mine")
            self.toCatchOre()
        else:
            rospy.logwarn("No Ore information, Robot will set to State: Observe")
            self.toObserve()

        
    def CatchOre(self):
        while not rospy.is_shutdown():
            self.statePub.publish("CatchOre")
            self.Rate.sleep()
            if self.isCatch:
                rospy.loginfo("Ore Catched!")
                break
        self.toTransportingOre()

    def TransportingOre(self):
        self.statePub.publish("TransportingOre")
        rospy.loginfo("State : TransportingOre")

        rospy.loginfo("Move to exchange station")
        self.move()
        self.toPlaneOre()

    def PlaneOre(self):
        self.statePub.publish("PlaneOre")
        rospy.loginfo("State : PlaneOre")
        while not rospy.is_shutdown():
            self.statePub.publish("PlaneOre")
            self.Rate.sleep()
            if self.isPlane:
                rospy.loginfo("To Next Ore")
                if self.isFinish:
                    rospy.loginfo("Ore Finished")
                    break
                else:
                    self.toFindingOre()

    def Back(self):
        self.move(0,0,0,0,0,0)

if __name__ == '__main__':
    m = Robotstates()
    m.init()
