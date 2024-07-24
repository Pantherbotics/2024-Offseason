// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    public static final String kMainCamName = "MainCam";
    public static final String kBackCamName = "BackCam";
    public static final String kNoteCamName = "notecam";

    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static final Transform3d kRobotToMainCam = new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, 0.0));
    public static final Transform3d kRobotToBackCam = new Transform3d(new Translation3d(0.0,0.0,0.0), new Rotation3d(0.0, 0.0, 0.0));
    public static final Transform3d kRobotToNoteCam = new Transform3d(new Translation3d(0.5,0.0,0.5), new Rotation3d(0.0, Units.degreesToRadians(45), 0.0));
}
