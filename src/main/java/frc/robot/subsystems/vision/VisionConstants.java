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
    public static final String kNoteCamName = "";

    public static final AprilTagFieldLayout kTagLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

    public static Transform3d kRobotToMainCam = new Transform3d(new Translation3d(Units.inchesToMeters(4),Units.inchesToMeters(-11),Units.inchesToMeters(46)), new Rotation3d(Units.degreesToRadians(5),0.0, 0.0));
    public static Transform3d kRobotToBackCam = new Transform3d(new Translation3d(Units.inchesToMeters(-10),Units.inchesToMeters(5.5),Units.inchesToMeters(6)), new Rotation3d(Units.degreesToRadians(180), Units.degreesToRadians(-53), Units.degreesToRadians(180)));
    public static Transform3d kRobotToNoteCam = new Transform3d(new Translation3d(Units.inchesToMeters(6),0.0,Units.inchesToMeters(23)), new Rotation3d(0.0, Units.degreesToRadians(36), 0.0));
}
