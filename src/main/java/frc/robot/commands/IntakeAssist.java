// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;

import org.opencv.core.Mat;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.NoteDetection;

public class IntakeAssist extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private DriverIO driverIO;


  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final IntegerArrayPublisher poses = inst.getIntegerArrayTopic("test").publish();

  public IntakeAssist(Intake m_intake, CommandSwerveDrivetrain m_drivetrain, DriverIO m_driverIO) {
    this.driverIO = m_driverIO;
    this.intake = m_intake;
    this.drivetrain = m_drivetrain;
    addRequirements(m_intake, m_drivetrain);
  }

  @Override
  public void initialize() {
    intake.setPivotDown();
    intake.setRollersIn();
  }

  private double[] toPolar(double x, double y){
    return new double[]{Math.hypot(x, y), Math.atan2(y, x)};
  }

  private double[] toCartesian(double[] polar){
    return new double[]{Math.sin(polar[1])*polar[0], Math.cos(polar[1])*polar[0]};
  }

  private Pose2d nearestNotePose(){
    return drivetrain.getState().Pose.nearest(Arrays.asList(NoteDetection.fieldNotes2d));
  }

  /*    Transform2d noteDiff = new Transform2d(nearestNote.getX()-robotPose.getX(), nearestNote.getY()-robotPose.getY(), Rotation2d.fromDegrees(0));
    double[] notePolar = toPolar(noteDiff.getX(), noteDiff.getY());
    double noteDist = notePolar[0];
    notePolar[0] = MathUtil.clamp(notePolar[0], 0, 1);
    double[] joyPolar = toPolar(driverIO.moveX(), driverIO.moveY());
    joyPolar[0] = MathUtil.clamp(joyPolar[0], 0, 1);
    double[] noteClamped = toCartesian(notePolar);
    double[] joyClamped = toCartesian(joyPolar);
    double[] difference = new double[]{noteClamped[0]-joyClamped[0], noteClamped[1]-joyClamped[1]};
    double angleDiff = MathUtil.clamp(notePolar[1]-robotPose.getRotation().getRadians(),-5,5) - MathUtil.clamp(driverIO.rotate(),-5,5);
    noteClamped = new double[]{noteClamped[0], noteClamped[1], MathUtil.clamp(notePolar[1]-robotPose.getRotation().getRadians(),-5,5)};
     */
  
  private void guidedDrive(){
    Pose2d robotPose = drivetrain.getState().Pose;
    Pose2d nearestNote = nearestNotePose();
    ChassisSpeeds speeds = drivetrain.getState().speeds;
    double[] joystickSpeeds = toPolar(-driverIO.moveX(), -driverIO.moveY());
    double[] toNote = toPolar(robotPose.getY() - nearestNote.getY(),robotPose.getX()-nearestNote.getX() );
    double match = Math.max(Math.cos(Math.abs(joystickSpeeds[1] - toNote[1])), 0);
    //match = Math.sqrt(match);

    double[] toNoteMove = toCartesian(new double[]{MathUtil.clamp(toNote[0], 0, 1), toNote[1]});
    Rotation2d toNoteRotate = Rotation2d.fromRadians(toNote[1]);
    toNoteRotate = toNoteRotate.minus(robotPose.getRotation());

    double xvel = MathUtil.interpolate(-driverIO.moveY(), toNoteMove[0] * joystickSpeeds[0], match);
    double yvel = MathUtil.interpolate(-driverIO.moveX(), toNoteMove[1] * joystickSpeeds[0], match);
    double rotationRate = MathUtil.interpolate(-driverIO.rotate(), toNoteRotate.getRadians() * joystickSpeeds[0], match);

    toNote = toCartesian(toNote);
    joystickSpeeds = toCartesian(joystickSpeeds);


      drivetrain.setControl(DriveConstants.drive
    .withVelocityX(xvel * DriveConstants.kMaxSpeed)
    .withVelocityY(yvel * DriveConstants.kMaxSpeed) 
    .withRotationalRate(-driverIO.rotate() * DriveConstants.kMaxAngularRate));
    poses.set(new long[]{Math.round(joystickSpeeds[0]*10), Math.round(joystickSpeeds[1]*10), Math.round(toNote[0]*10), Math.round(toNote[1]*10), Math.round(match*10), 0});
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    guidedDrive();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
