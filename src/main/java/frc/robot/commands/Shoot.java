// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.vision.FieldPoses;

public class Shoot extends Command {
  private final Shooter shooter;
  private final CommandSwerveDrivetrain drivetrain;
  private final DriverIO mainIO;
  private boolean shootButton = true;
  private boolean justShot = false;
  private noteStates noteState;
  private RegionIn regionIn;
  private Pose2d targetPose;
  private InterpolatingDoubleTreeMap map;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Pose");
  private final StructPublisher<Pose2d> targetPub = table.getStructTopic("targetPose", Pose2d.struct).publish();

  private enum noteStates{
    SEATED,     // (  |)|    the bars || are the sensor and the parentheses are the ends of the note
    SHOOTING,   // ( || )
    END,        // |(|  )
    OUT         // ||(  )
  }
  private enum RegionIn{
    SHOOT,
    AMP_PASS,
    MID_PASS
  }
  private Point pointFromPose(Pose2d pose){
    return new Point(pose.getX(), pose.getY());
  }

  public Shoot(Shooter shooter, CommandSwerveDrivetrain drivetrain, DriverIO mainIO) {
    this.shooter = shooter;
    this.drivetrain = drivetrain;
    this.mainIO = mainIO;
    addRequirements(shooter, drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setFlywheelSpeed(ShooterConstants.kFlywheelShotSpeed);
    shooter.setRollers(0);
    shootButton = true;
    justShot = false;
    noteState = noteStates.SEATED;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!shootButton && mainIO.shoot().getAsBoolean()){
      justShot = true;
    }

    if (justShot) {
      shooter.setRollers(ShooterConstants.kRollersShootSpeed);
    }

    if (noteState == noteStates.SEATED && !shooter.noteSeated()){
      noteState = noteStates.SHOOTING;
    } else if (noteState == noteStates.SHOOTING && shooter.noteSeated()) {
      noteState = noteStates.END;
    } else if (noteState == noteStates.END && !shooter.noteSeated()) {
      noteState = noteStates.OUT;
    }


    Pose2d robotPose = drivetrain.getState().Pose;
    Point robotPoint = pointFromPose(robotPose);
    if (FieldPoses.kShootRegion.contains(robotPoint)){
      regionIn = RegionIn.SHOOT;
    } else if (FieldPoses.kAmpPassRegion.contains(robotPoint)){
      regionIn = RegionIn.AMP_PASS;
    } else if( FieldPoses.kMidPassRegion.contains(robotPoint)){
      regionIn = RegionIn.MID_PASS;
    } else {
      regionIn = RegionIn.SHOOT;
    }

    switch(regionIn){
      case SHOOT:
      targetPose = FieldPoses.kSpeakerPose;
      map = shooter.getShotTable();
      break;
      case AMP_PASS:
      targetPose = FieldPoses.kAmpPassPose;
      map = shooter.getPassTable();
      break;
      case MID_PASS:
      targetPose = FieldPoses.kMidPassPose;
      map = shooter.getPassTable();
      break;
    }
    
    Rotation2d angleToSpeaker = new Rotation2d(targetPose.getX() - robotPose.getX(),targetPose.getY() - robotPose.getY()).plus(Rotation2d.fromDegrees(180));//Rotation2d.fromRadians(Math.atan2(targetPose.getY() - robotPose.getY(), targetPose.getX() - robotPose.getX()));
    double distanceToSpeaker = robotPose.relativeTo(targetPose).getTranslation().getNorm();
    drivetrain.setControl(
      DriveConstants.facing.withTargetDirection(angleToSpeaker)
      .withVelocityX(-mainIO.moveY() * DriveConstants.kMaxSpeed)
      .withVelocityY(-mainIO.moveX() * DriveConstants.kMaxSpeed)
      );

    shooter.setPivotGoal(map.get(distanceToSpeaker));
    
    targetPub.set(targetPose);
    SmartDashboard.putString("region", regionIn.toString());
    
    shootButton = mainIO.shoot().getAsBoolean();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRollers(0);
    shooter.setFlywheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return noteState == noteStates.OUT;
  }
}
