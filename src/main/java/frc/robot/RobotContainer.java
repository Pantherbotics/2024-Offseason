// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Amp;
import frc.robot.commands.Handoff;
import frc.robot.commands.IntakeAssist;
import frc.robot.commands.Shoot;
import frc.robot.commands.climb;
import frc.robot.commands.simpleShot;
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
  private final CommandXboxController mainController;
  private final CommandSwerveDrivetrain drivetrain;
  private final Vision vision;
  private final NoteDetection noteDetection;
  private final Climber climber;
  private final Shooter shooter;
  private final Intake intake;
  private final Telemetry logger = new Telemetry();

  private final SendableChooser <Command> autoChooser;

  public RobotContainer() {
    drivetrain = TunerConstants.DriveTrain;
    drivetrain.registerTelemetry(logger::telemeterize);
    vision = new Vision();
    noteDetection = new NoteDetection(drivetrain);
    mainController = new CommandXboxController(0);
    
    shooter = new Shooter();
    intake = new Intake();
    climber = new Climber();

    drivetrain.setDefaultCommand(DriveConstants.driveCommand(drivetrain, mainIO).ignoringDisable(true));
    configureBindings();
    drivetrain.seedFieldRelative(new Pose2d(6.0, 4.0, Rotation2d.fromDegrees(0)));
    vision.setDefaultCommand(vision.updatePose(drivetrain).ignoringDisable(true));
    drivetrain.configNeutralMode(NeutralModeValue.Coast);

    invertEncoders();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", this.autoChooser);
  }

  public void onStart(){
    CommandScheduler.getInstance().schedule(intake.zeroIntake(), shooter.rollersStop());
  }

  private void configureBindings() {
    
    /*
    mainController.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
    mainController.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    mainController.button(1).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    mainController.button(2).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    mainController.button(3).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    mainController.button(4).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        */
    
    mainIO.intake().toggleOnTrue(new IntakeAssist(intake, drivetrain, mainIO));
    mainIO.climb().onTrue(new climb(climber));
    mainIO.shoot().onTrue(new Shoot(shooter, drivetrain, mainIO));
    mainIO.amp().onTrue(new Amp(shooter, drivetrain, mainIO));
    mainIO.reset().onTrue(Commands.idle(shooter, intake));

    intake.gotNote().onTrue(new Handoff(intake, shooter).withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    

    
        
  }


private void invertEncoders(){
  if(!Utils.isSimulation()){
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
    return autoChooser.getSelected();
  }
}
