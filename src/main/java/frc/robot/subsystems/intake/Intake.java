// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor;
  private final TalonFX m_rollersMotor;
  private final DigitalInput m_limitSwitch;
  private final AnalogInput m_distanceSensor;

  private ExponentialProfile profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(10, IntakeConstants.Kv, IntakeConstants.Ka));
  private ArmFeedforward feedforward = new ArmFeedforward(IntakeConstants.Ks, IntakeConstants.Kg, IntakeConstants.Kv, IntakeConstants.Ka);
  private PIDController controller = new PIDController(IntakeConstants.Kp, IntakeConstants.Ki, IntakeConstants.Kd, IntakeConstants.dt);

  private ExponentialProfile.State currentSetpoint = new ExponentialProfile.State();
  private ExponentialProfile.State goal = new ExponentialProfile.State();

  public Intake() {
    m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID);
    m_rollersMotor = new TalonFX(IntakeConstants.kRollersMotorID);
    m_limitSwitch = new DigitalInput(IntakeConstants.kLimitSiwtchID);
    m_distanceSensor = new AnalogInput(IntakeConstants.kDistanceSensorID);
  }

  public boolean limitSwitch(){
    return m_limitSwitch.get();
  }

  public boolean hasNote(){
    return m_distanceSensor.getAverageValue() < IntakeConstants.kSensorThreshold;
  }

  public void setGoal(double goal){
    this.goal.position = goal;
  }

  public boolean isAtGoal(double tolerance){
    return MathUtil.isNear(this.goal.position, m_pivotMotor.get(), tolerance);
  }

  public void setRollers(double speed){
    m_rollersMotor.set(speed);
  }

  public void setPivotDown(){
    setGoal(IntakeConstants.kDownPosition);
  }

  public void setPivotUp(){
    setGoal(IntakeConstants.kUpPosition);
  }

  public void setRollersIn(){
    setRollers(IntakeConstants.kInSpeed);
  }

  public void setRollersStop(){
    setRollers(0);
  }

  public void setRollersOut(){
    setRollers(IntakeConstants.kOutSpeed);
  }

  public Command pivotDown(){
    return runOnce(()->setPivotDown());
  }

  public Command pivotUp(){
    return runOnce(()->setPivotUp());
  }

  public Command rollersIn(){
    return runOnce(()->setRollersIn());
  }

  public Command rollersStop(){
    return runOnce(()->setRollersStop());
  }

  public Command rollersOut(){
    return runOnce(()->setRollersOut());
  }

  @Override
  public void periodic() {
    var nextSetpoint = profile.calculate(IntakeConstants.dt, currentSetpoint, goal);

    m_pivotMotor.setVoltage(
        feedforward.calculate(currentSetpoint.velocity, nextSetpoint.velocity, IntakeConstants.dt)
            + controller.calculate(m_pivotMotor.get(), currentSetpoint.position));

    currentSetpoint = nextSetpoint;
  }
}