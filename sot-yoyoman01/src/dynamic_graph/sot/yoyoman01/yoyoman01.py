from __future__ import print_function

import numpy as np

from dynamic_graph.sot.dynamics_pinocchio.humanoid_robot import AbstractHumanoidRobot
from dynamic_graph.sot.dynamics_pinocchio import DynamicPinocchio
from dynamic_graph import plug

from dynamic_graph.ros import RosRobotModel
import pinocchio as se3
from rospkg import RosPack

# Internal helper tool.
def matrixToTuple(M):
    tmp = M.tolist()
    res = []
    for i in tmp:
        res.append(tuple(i))
    return tuple(res)

class Yoyoman01(AbstractHumanoidRobot):
    """
    This class defines a Talos robot
    """

#    forceSensorInLeftAnkle =  ((1.,0.,0.,0.),
#                               (0.,1.,0.,0.),
#                               (0.,0.,1.,-0.107),
#                               (0.,0.,0.,1.))
#    forceSensorInRightAnkle = ((1.,0.,0.,0.),
#                               (0.,1.,0.,0.),
#                               (0.,0.,1.,-0.107),
#                               (0.,0.,0.,1.))
    """
    TODO: Confirm the position and existence of these sensors
    accelerometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))

    gyrometerPosition = np.matrix ((
            (1., 0., 0., -.13,),
            (0., 1., 0., 0.,),
            (0., 0., 1., .118,),
            (0., 0., 0., 1.,),
            ))
    """
    def smallToFull(self, config):
        #Gripper position in full configuration: 27:34, and 41:48
        #Small configuration: 36 DOF
        #Full configuration: 50 DOF
        res = config[0:27] + 7*(0.,) + config[27:34]+ 7*(0.,)+config[34:]
        return res

    def __init__(self, name, initialConfig, device = None, tracer = None):
        self.OperationalPointsMap = {'left-wrist'  : 'Larm',
                                     'right-wrist' : 'Rarm',
                                     'left-ankle'  : 'LHip',
                                     'right-ankle' : 'RHip',
                                     'gaze'        : 'Head',
                                     'waist'       : 'root_joint',
                                     'chest'       : 'base_link'}

        from rospkg import RosPack
        rospack = RosPack()
        urdfPath = rospack.get_path('yoyoman01_sot_description')+"/urdf/yoyoman01.urdf"
        urdfDir = [rospack.get_path('yoyoman01_sot_description')+"/../"]

        # Create a wrapper to access the dynamic model provided through an urdf file.
        from pinocchio.robot_wrapper import RobotWrapper
        import pinocchio as se3
        from dynamic_graph.sot.dynamics_pinocchio import fromSotToPinocchio
        pinocchioRobot = RobotWrapper(urdfPath, urdfDir, se3.JointModelFreeFlyer())
        self.pinocchioModel = pinocchioRobot.model
        self.pinocchioData = pinocchioRobot.data

        AbstractHumanoidRobot.__init__ (self, name, tracer)

        self.OperationalPoints.append('waist')
        self.OperationalPoints.append('chest')

        # Create rigid body dynamics model and data (pinocchio) 
        self.dynamic = DynamicPinocchio(self.name + "_dynamic")
        self.dynamic.setModel(self.pinocchioModel)
        self.dynamic.setData(self.pinocchioData)
        self.dimension = self.dynamic.getDimension()

        # Initialize device
        self.device = device

        self.device.resize(self.dynamic.getDimension())
        self.halfSitting = initialConfig
        self.device.set(self.halfSitting)
        plug(self.device.state, self.dynamic.position)

#        self.AdditionalFrames.append(
#            ("leftFootForceSensor",
#             self.forceSensorInLeftAnkle, self.OperationalPointsMap["left-ankle"]))
#        self.AdditionalFrames.append(
#            ("rightFootForceSensor",
#             self.forceSensorInRightAnkle, self.OperationalPointsMap["right-ankle"]))
        
        self.dimension = self.dynamic.getDimension()
        self.plugVelocityFromDevice = True
        self.dynamic.displayModel()

        # Initialize velocity derivator if chosen
        if self.enableVelocityDerivator:
            self.velocityDerivator = Derivator_of_Vector('velocityDerivator')
            self.velocityDerivator.dt.value = self.timeStep
            plug(self.device.state, self.velocityDerivator.sin)
            plug(self.velocityDerivator.sout, self.dynamic.velocity)
        else:
            self.dynamic.velocity.value = self.dimension*(0.,)

        # Initialize acceleration derivator if chosen            
        if self.enableAccelerationDerivator:
            self.accelerationDerivator = \
                Derivator_of_Vector('accelerationDerivator')
            self.accelerationDerivator.dt.value = self.timeStep
            plug(self.velocityDerivator.sout,
                 self.accelerationDerivator.sin)
            plug(self.accelerationDerivator.sout, self.dynamic.acceleration)
        else:
            self.dynamic.acceleration.value = self.dimension*(0.,)

        # Create operational points based on operational points map (if provided)
        if self.OperationalPointsMap is not None:
            self.initializeOpPoints()

__all__ = [Yoyoman01]
