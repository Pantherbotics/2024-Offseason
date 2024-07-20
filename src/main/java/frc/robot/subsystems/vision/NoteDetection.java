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

public class NoteDetection extends SubsystemBase {
  
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");

  public static Pose2d[] fieldNotes2d;

  StructArrayPublisher<Pose3d> notePublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("fieldNotes3d", Pose3d.struct).publish();


  public NoteDetection() {}


  public Transform2d noteAngleToTranslation(double x, double y){
    double camHeight = VisionConstants.kRobotToNoteCam.getRotation().getY() - Units.inchesToMeters(1);
    double pitchAngle = VisionConstants.kRobotToNoteCam.getRotation().getY() + y; // true angle relative to robot

    double xDist = (camHeight) / Math.tan(pitchAngle); // x is forward relative to robot
    double yDist = (camHeight) / Math.sin(pitchAngle) * Math.sin(x); // y is left relative to robot
    return new Transform2d(xDist, yDist, new Rotation2d());
  }


  public void getNotes(Supplier<Pose2d> robotPose){

    double[] noteX = tx.getDoubleArray(new double[]{});
    double[] noteY = ty.getDoubleArray(new double[]{});

    Transform2d[] robotRelativeNotes = new Transform2d[noteX.length];
    fieldNotes2d = new Pose2d[noteX.length];
    Pose3d[] fieldNotes3d = new Pose3d[noteX.length];

    for (var i = 0; i < noteX.length; i++){
      robotRelativeNotes[i] = noteAngleToTranslation(noteX[i], noteY[i]);
      fieldNotes2d[i] = robotPose.get().plus(robotRelativeNotes[i]);
      fieldNotes3d[i] = new Pose3d(fieldNotes2d[i].getX(), fieldNotes2d[i].getY(), 0.0, new Rotation3d(0.0,0.0,0.0));
    }
    notePublisher.set(fieldNotes3d);
  }

  
  public Command findNotes (Supplier<Pose2d> robotPose) {
    return run(()->getNotes(robotPose));
  }


  @Override
  public void periodic() {

  }
}
