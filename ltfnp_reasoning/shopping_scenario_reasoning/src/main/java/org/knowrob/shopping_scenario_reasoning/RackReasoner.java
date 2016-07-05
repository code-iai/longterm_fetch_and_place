/*
 * RackReasoner.java
 * Copyright (c) 2015 Jan Winkler
 *
 * All rights reserved.
 *
 * Software License Agreement (BSD License)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Universitaet Bremen nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

package org.knowrob.ltfnp_reasoning;

import java.util.*;
import java.util.Map.*;
import java.lang.Integer;

import org.knowrob.utils.ros.RosUtilities;


public class RackReasoner {
    /** 
     *  Determined whether the position [dPoseX, dPoseY, dPoseZ] is on
     *  the rack level whose position is [dRackLevelX, dRackLevelY,
     *  dRackLevelZ], and its two dimensional extents are
     *  [dLevelWidth, dLevelDepth]. dLevelHeight determines the
     *  tolerance area above the rack level's plane in which positions
     *  are considered to be on the level.
     */
    public boolean positionOnRackLevel(double dPoseX, double dPoseY, double dPoseZ, double dRackLevelX, double dRackLevelY, double dRackLevelZ, double dLevelWidth, double dLevelDepth, double dLevelHeight) {
	return (dPoseX >= dRackLevelX - dLevelDepth / 2 && dPoseX < dRackLevelX + dLevelDepth / 2 &&
		dPoseY >= dRackLevelY - dLevelWidth / 2 && dPoseY < dRackLevelY + dLevelWidth / 2 &&
		dPoseZ >= dRackLevelZ && dPoseZ < dRackLevelZ + dLevelHeight);
    }
    
    /**
     *  Calculate the surface elevation of a rack level, given its
     *  center elevation and its height.
     */
    public double rackLevelElevation(double dZ, double dHeight) {
	return dZ + (dHeight / 2);
    }
    
    /**
     *  Absolute position of relative coordinates (x, y, z in a double
     *  array) on a rack level surface, given the absolute center
     *  position of the rack level, and the relative coordinates (x,
     *  y). The relative coordinates are relative to the center of the
     *  rack level.
     */
    public double[] rackLevelRelativePosition(double dX, double dY, double dZ, double dRelativeX, double dRelativeY) {
	double[] arrReturn = {dX + dRelativeX, dY + dRelativeY, dZ};
	
	return arrReturn;
    }
    
    /**
     *  Resolves a relative package path. Paths like
     *  `package://my_package/models` are resolved into the absolute
     *  path of the package `my_package`, plus `/models` appended.
     */
    public String resolveRelativePath(String strRelativePath) {
	String strReturn = strRelativePath;
	
	if(strRelativePath.substring(0, 10).equals("package://")) {
	    int nNextSlash = strRelativePath.indexOf('/', 11);
	    String strPkg = strRelativePath.substring(10, nNextSlash);
	    
	    String strAbs = RosUtilities.rospackFind(strPkg);
	    
	    strReturn = strAbs + strRelativePath.substring(nNextSlash);
	}
	
	return strReturn;
    }
}
