# -*- coding: utf-8 -*-
# Copyright 2016, Olivier Stasse, CNRS
#
# This file is part of sot-talos.
# sot-talos is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# sot-talos is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# sot-talos. If not, see <http://www.gnu.org/licenses/>.


from dynamic_graph.sot.yoyoman01.yoyoman01 import Yoyoman01
import numpy as np

class Robot (Yoyoman01):
    """
    This class instantiates LAAS Yoyoman01 Robot
    """


    halfSitting = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    def __init__(self, name,
                 device = None,
                 tracer = None):
        Yoyoman01.__init__(self,name,self.halfSitting,device,tracer)
        """
        TODO:Confirm these values
        # Define camera positions w.r.t gaze.
        cameraBottomLeftPosition = np.matrix((
            (0.98481, 0.00000, 0.17365, 0.035),
            (0.,      1.,      0.,      0.072),
            (-0.17365, 0.00000, 0.98481, 0.075 - 0.03),
            (0., 0., 0., 1.),
        ))
        cameraBottomRightPosition = np.matrix((
            (0.98481, 0.00000, 0.17365, 0.035),
                (0.,      1.,      0.,     -0.072),
                (-0.17365, 0.00000, 0.98481, 0.075 - 0.03),
                (0., 0., 0., 1.),
                ))
        cameraTopLeftPosition = np.matrix((
            (0.99920,  0.00120, 0.03997, 0.01),
            (0.00000,  0.99955,-0.03000, 0.029),
            (-0.03999, 0.02997, 0.99875, 0.145 - 0.03),
            (0.,       0.,      0.,      1.),
        ))
        cameraTopRightPosition = np.matrix((
            (0.99920,  0.00000, 0.03999,  0.01),
            (0.00000,  1.00000, 0.00000, -0.029),
            (-0.03999, 0.00000, 0.99920,  0.145 - 0.03),
            (0.,       0.,      0.,       1.),
        ))
        # Frames re-orientation: 
        # Z = depth (increase from near to far)
        # X = increase from left to right
        # Y = increase from top to bottom

        c1_M_c = np.matrix(
            [[ 0.,  0.,  1., 0.],
             [-1.,  0.,  0., 0.],
             [ 0., -1.,  0., 0.],
             [ 0.,  0.,  0., 1.]])
        
        for camera in [cameraBottomLeftPosition, cameraBottomRightPosition,
                       cameraTopLeftPosition, cameraTopRightPosition]:
            camera[:] = camera * c1_M_c

        self.AdditionalFrames.append(
            ("cameraBottomLeft",
             matrixToTuple(cameraBottomLeftPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraBottomRight",
             matrixToTuple(cameraBottomRightPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraTopLeft",
             matrixToTuple(cameraTopLeftPosition), "gaze"))
        self.AdditionalFrames.append(
            ("cameraTopRight",
             matrixToTuple(cameraTopRightPosition), "gaze"))
        """

__all__ = ["Robot"]
