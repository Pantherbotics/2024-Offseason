// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.IntakeAssist;
import frc.robot.controls.DriverIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.DriveConstants;
import frc.robot.subsystems.drivetrain.Telemetry;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.ShooterPivot;
import frc.robot.subsystems.vision.NoteDetection;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  
  public static final DriverIO driverIO = new DriverIO(0);
  private final CommandJoystick mainController;
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final NoteDetection noteDetection;
  private final ShooterPivot shooterPivot;
  private final Intake intake;
  private final Climber climber;
  private final Telemetry logger = new Telemetry(DriveConstants.kMaxSpeed);

  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    drivetrain.registerTelemetry(logger::telemeterize);
    vision = new Vision();
    noteDetection = new NoteDetection();
    mainController = new CommandJoystick(0);
    
    shooterPivot = new ShooterPivot();
    intake = new Intake();
    climber = new Climber();

    drivetrain.setDefaultCommand(DriveConstants.driveCommand(drivetrain, mainController.getHID()).ignoringDisable(true));
    configureBindings();
    vision.setDefaultCommand(vision.updatePose(drivetrain));
    noteDetection.setDefaultCommand(noteDetection.findNotes(()->drivetrain.getState().Pose));

    invertEncoders();

  }

  private void configureBindings() {
    mainController.button(1).onTrue(drivetrain.pathfindToPosition(DriveConstants.kAmpPose));
    /*mainController.button(1).whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kForward));
    mainController.button(2).whileTrue(shooterPivot.sysIdDynamicCommand(Direction.kReverse));
    mainController.button(3).whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kForward));
    mainController.button(4).whileTrue(shooterPivot.sysIdQuasistaticCommand(Direction.kReverse));
    */
    mainController.button(2).onTrue(new IntakeAssist(intake, drivetrain, driverIO));
    mainController.pov(0).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    mainController.pov(90).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    mainController.pov(180).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    mainController.pov(270).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
  }

  private void invertEncoders(){
    if (!Utils.isSimulation()){
    for (int i = 0; i < 4; ++i)
    {
      var module = drivetrain.getModule(i);
      CANcoderConfiguration cfg = new CANcoderConfiguration();
      StatusCode response = StatusCode.StatusCodeNotInitialized;

      /* Repeat this in a loop until we have success */
      do {
        /* First make sure we refresh the object so we don't overwrite anything */
        response = module.getCANcoder().getConfigurator().refresh(cfg);
      } while(!response.isOK());

      /* Invert your CANcoder magnet direction */
      cfg.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

      /* Repeat this in a loop until we have success */
      do {
        /* Apply configuration to CANcoder */
        module.getCANcoder().getConfigurator().apply(cfg);
      } while (!response.isOK());
    }
  }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
