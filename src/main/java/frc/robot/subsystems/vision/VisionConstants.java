// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {
    public static String mainCamName = "MainCam";
    public static String backCamName = "BackCam";
    public static String noteCamName = "BackCam";

    public static Transform3d mainCamToRobot = new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, 0.0));
    public static Transform3d backCamToRobot = new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, 0.0));
    public static Transform3d noteCamToRobot = new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, 0.0));
}
