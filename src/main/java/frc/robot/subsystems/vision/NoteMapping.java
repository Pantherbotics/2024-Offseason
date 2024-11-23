// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class NoteMapping {

    public void run(){
        double topAngle;
        double bottomAngle;
        double camAngle = VisionConstants.kRobotToNoteCam.getRotation().getY();
        double camHeight = VisionConstants.kRobotToNoteCam.getZ();
        LimelightHelpers.RawDetection[] llresults = LimelightHelpers.getRawDetections("");
        for (var i = 0; i < llresults.length; i++){
            topAngle =llresults[i].corner0_Y;
            bottomAngle = llresults[i].corner2_Y;
            topAngle = Units.degreesToRadians(topAngle) + camAngle;
            bottomAngle = Units.degreesToRadians(bottomAngle) + camAngle;
            double topDist = -camHeight * (1/Math.tan(topAngle)) + 1/Math.tan(topAngle/2);
            double bottomDist = -camHeight * (1/Math.tan(bottomAngle)) - Math.tan(bottomAngle/2);
            double avgDist = (topDist+bottomDist)/2;

        }
    }
}
