// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.vision.NoteDetection;

public class IntakeAssist extends Command {
  private final Intake intake;
  private final CommandSwerveDrivetrain drivetrain;
  private DriverIO mainIO;


  //private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  //private final IntegerArrayPublisher poses = inst.getIntegerArrayTopic("test").publish();

  public IntakeAssist(Intake m_intake, CommandSwerveDrivetrain m_drivetrain, DriverIO m_mainIO) {
    this.mainIO = m_mainIO;
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

  private void guidedDrive(){
    double xvel;
    double yvel;
    double rotationRate;
    if (NoteDetection.fieldNotes2d.length > 0){
      Pose2d robotPose = drivetrain.getState().Pose;
      Pose2d nearestNote = nearestNotePose();
      double[] joystickSpeeds = toPolar(-mainIO.moveX(), -mainIO.moveY());
      double[] toNote = toPolar(robotPose.getY() - nearestNote.getY(),robotPose.getX()-nearestNote.getX() );
      double match = Math.max(Math.cos(Math.abs(joystickSpeeds[1] - toNote[1])), 0);
      match = Math.sqrt(match);
      //match /= Math.cbrt(toNote[0]);

      double[] toNoteMove = toCartesian(new double[]{MathUtil.clamp(toNote[0], 0.1, 1), toNote[1]});
      Rotation2d toNoteRotate = Rotation2d.fromRadians(-toNote[1]);
      toNoteRotate = toNoteRotate.minus(robotPose.getRotation().plus(Rotation2d.fromDegrees(90)));
      toNoteRotate = toNoteRotate.div(2);
      double rotate = toNoteRotate.getRadians();
      rotate += MathUtil.clamp(Math.copySign(0.1, rotate),-1,1);
      
      xvel = MathUtil.interpolate(-mainIO.moveY(), toNoteMove[0] * joystickSpeeds[0], match);
      yvel = MathUtil.interpolate(-mainIO.moveX(), toNoteMove[1] * joystickSpeeds[0], match);
      rotationRate = MathUtil.interpolate(-mainIO.rotate(), rotate * joystickSpeeds[0], match);

      mainIO.setRumble(1.2/Math.pow(toNote[0], 2));

      toNote = toCartesian(toNote);
      joystickSpeeds = toCartesian(joystickSpeeds);
    } else {
      xvel = -mainIO.moveY();
      yvel = -mainIO.moveX();
      rotationRate = -mainIO.rotate();
    }

      drivetrain.setControl(DriveConstants.drive
    .withVelocityX(xvel * DriveConstants.kMaxSpeed)
    .withVelocityY(yvel * DriveConstants.kMaxSpeed) 
    .withRotationalRate(rotationRate * DriveConstants.kMaxAngularRate));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    guidedDrive();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setRollersStop();
    intake.setPivotUp();
    mainIO.setRumble(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.hasNote();
  }
}
