// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

public class NoteDetection extends SubsystemBase {
  
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private CommandSwerveDrivetrain drivetrain;

  public static Pose2d[] fieldNotes2d;

  StructArrayPublisher<Pose3d> notePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("fieldNotes3d", Pose3d.struct).publish();


  public NoteDetection(CommandSwerveDrivetrain m_drivetrain) {
    drivetrain = m_drivetrain;
  }


  public Transform2d noteAngleToTranslation(double x, double y){
    double camHeight = VisionConstants.kRobotToNoteCam.getRotation().getY() - Units.inchesToMeters(1);
    double pitchAngle = VisionConstants.kRobotToNoteCam.getRotation().getY() + y; // true angle relative to robot

    double xDist = (camHeight) / Math.tan(pitchAngle); // x is forward relative to robot
    double yDist = (camHeight) / Math.sin(pitchAngle) * Math.sin(x); // y is left relative to robot
    return new Transform2d(xDist, yDist, new Rotation2d());
  }


  public void getNotes(){
    LimelightHelpers.LimelightResults llresults = LimelightHelpers.getLatestResults("");
    LimelightHelpers.LimelightTarget_Detector[] results = llresults.targets_Detector;

    double[] noteX = tx.getDoubleArray(new double[]{});
    double[] noteY = ty.getDoubleArray(new double[]{});

    System.out.println(results.toString());
    Transform2d[] robotRelativeNotes = new Transform2d[results.length];
    fieldNotes2d = new Pose2d[]{};//new Pose2d(5,5,Rotation2d.fromDegrees(0))};
    Pose3d[] fieldNotes3d = new Pose3d[]{};//new Pose3d(5,5,Units.inchesToMeters(1), new Rotation3d(0.0,0.0,0.0))};

    for (var i = 0; i < results.length; i++){
      robotRelativeNotes[i] = noteAngleToTranslation(results[i].tx, results[i].ty);
      fieldNotes2d[i] = drivetrain.getState().Pose.plus(robotRelativeNotes[i]);
      fieldNotes3d[i] = new Pose3d(fieldNotes2d[i].getX(), fieldNotes2d[i].getY(), Units.inchesToMeters(1), new Rotation3d(0.0,0.0,0.0));
    }

    notePublisher.set(fieldNotes3d);
  }


  @Override
  public void periodic() {
    getNotes();
  }
}
