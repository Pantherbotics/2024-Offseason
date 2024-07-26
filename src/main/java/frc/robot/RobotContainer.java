// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.commands.Handoff;
import frc.robot.commands.IntakeAssist;
import frc.robot.commands.Shoot;
import frc.robot.controls.DriverIO;
import frc.robot.controls.ControlConstants.InputType;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.NoteDetection;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  
  public static final DriverIO mainIO = new DriverIO(0, InputType.XBOX);
  private final CommandJoystick mainController;
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final NoteDetection noteDetection;
  private final Shooter shooter;
  private final Intake intake;
  private final Climber climber;
  private final Telemetry logger = new Telemetry();

  private final SendableChooser <Command> autoChooser;

  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    drivetrain.registerTelemetry(logger::telemeterize);
    vision = new Vision();
    noteDetection = new NoteDetection(drivetrain);
    mainController = new CommandJoystick(0);
    
    shooter = new Shooter();
    intake = new Intake();
    climber = new Climber();

    drivetrain.setDefaultCommand(DriveConstants.driveCommand(drivetrain, mainIO).ignoringDisable(true));
    configureBindings();
    vision.setDefaultCommand(vision.updatePose(drivetrain));



    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  private void configureBindings() {
    
    /*mainController.button(1).whileTrue(shooter.sysIdDynamicCommand(Direction.kForward));
    mainController.button(2).whileTrue(shooter.sysIdDynamicCommand(Direction.kReverse));
    mainController.button(3).whileTrue(shooter.sysIdQuasistaticCommand(Direction.kForward));
    mainController.button(4).whileTrue(shooter.sysIdQuasistaticCommand(Direction.kReverse));
    
    mainController.button(2).onTrue(new IntakeAssist(intake, drivetrain, mainIO));
    mainController.pov(0).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    mainController.pov(90).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    mainController.pov(180).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    mainController.pov(270).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    */
    mainIO.intake().toggleOnTrue(new IntakeAssist(intake, drivetrain, mainIO));
    //mainIO.climb().onTrue(climber.climbUntilSwitches());
    mainIO.shoot().onTrue(new Shoot(shooter, drivetrain, mainIO));
    mainIO.climb().onTrue(new Handoff(intake, shooter));
    intake.gotNote().onTrue(new Handoff(intake, shooter));
        
  }





  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
