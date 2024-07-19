// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NoteDetection extends SubsystemBase {
  
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");

  public NoteDetection() {}

  public Translation2d noteAngleToTranslation(double x, double y){
    // from https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance
    double camHeight = VisionConstants.kRobotToNoteCam.getRotation().getY() - Units.inchesToMeters(1);
    double pitchAngle = VisionConstants.kRobotToNoteCam.getRotation().getY() + y; // true angle relative to robot

    double xDist = (camHeight) / Math.tan(pitchAngle); // x is forward relative to robot
    double yDist = (camHeight) / Math.sin(pitchAngle) * Math.sin(x); // y is left relative to robot
    return new Translation2d(xDist, yDist);
  }

  public void findNotes(){
    var noteX = tx.getDoubleArray(new double[]{});
    var noteY = ty.getDoubleArray(new double[]{});
    for (var i = 0; i < noteX.length; i++){

    }
  }

  @Override
  public void periodic() {

  }
}
