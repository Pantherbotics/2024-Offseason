// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Shooter extends SubsystemBase {
  private final TalonFX m_pivotMotor;
  private final DutyCycleEncoder m_encoder;

  private ExponentialProfile profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, ShooterConstants.Kv, ShooterConstants.Ka));
  private ArmFeedforward feedforward = new ArmFeedforward(ShooterConstants.Ks, ShooterConstants.Kg, ShooterConstants.Kv, ShooterConstants.Ka);
  private PIDController controller = new PIDController(ShooterConstants.Kp, ShooterConstants.Ki, ShooterConstants.Kd, ShooterConstants.dt);

  private ExponentialProfile.State currentSetpoint = new ExponentialProfile.State();
  private ExponentialProfile.State goal = new ExponentialProfile.State();

  private final SysIdRoutine routine = new SysIdRoutine(new Config(), new Mechanism(this::pivotVoltage, null, this));
  
  public Shooter() {

    m_encoder = new DutyCycleEncoder(ShooterConstants.kEncoderID);
    m_encoder.setPositionOffset(ShooterConstants.kEncoderOffset);
    m_pivotMotor = new TalonFX(ShooterConstants.kPivotMotorID);

    setGoal(ShooterConstants.kHandoffPosition);
    
  }

  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setVoltage(voltageMeasure.magnitude());
  }

  public void setGoal(double goal){
    this.goal.position = goal;
  }

  
  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }

  @Override
  public void periodic() {
    var nextSetpoint = profile.calculate(ShooterConstants.dt, currentSetpoint, goal);

    m_pivotMotor.setVoltage(
        feedforward.calculate(Units.rotationsToRadians(m_encoder.get()), Units.rotationsToRadians(m_pivotMotor.getVelocity().getValueAsDouble()), Units.rotationsToRadians(m_pivotMotor.getAcceleration().getValueAsDouble()))
            + controller.calculate(m_encoder.get(), currentSetpoint.position));

    currentSetpoint = nextSetpoint;
  }
}
