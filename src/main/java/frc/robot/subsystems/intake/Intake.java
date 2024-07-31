// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor;
  private final TalonFX m_rollersMotor;
  private final DigitalInput m_limitSwitch;
  private final AnalogInput m_distanceSensor;
  private final MotionMagicVoltage m_request;

  
  private final SysIdRoutine routine = new SysIdRoutine(new Config(), new Mechanism(this::pivotVoltage, null, this));
  
  public Intake() {
    m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID);
    m_rollersMotor = new TalonFX(IntakeConstants.kRollersMotorID);
    m_limitSwitch = new DigitalInput(IntakeConstants.kLimitSiwtchID);
    m_distanceSensor = new AnalogInput(IntakeConstants.kDistanceSensorID);
    m_distanceSensor.setAverageBits(4);
    m_pivotMotor.setPosition(0);

    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration();
    pivotConfigs.withSlot0(IntakeConstants.kPivotGains);
    pivotConfigs.withMotionMagic(IntakeConstants.kProfileConfigs);

    FeedbackConfigs feedbackConfigs = pivotConfigs.Feedback;
    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kMotorToPivotRatio);
    

    m_pivotMotor.getConfigurator().apply(pivotConfigs);
    m_request = new MotionMagicVoltage(0);
    

    SmartDashboard.putData("Intake", this);
  }

    
  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setVoltage(voltageMeasure.magnitude());
  }

  public boolean limitSwitch(){
    return m_limitSwitch.get();
  }

  public boolean hasNote(){
    return m_distanceSensor.getAverageValue() > IntakeConstants.kSensorThreshold;
  }

  public void setGoal(double goal){
    SmartDashboard.putNumber("Intake goal", goal);
    m_pivotMotor.setControl(m_request.withPosition(goal));
  }

  public boolean isAtGoal(){
    return Math.abs(m_request.Position - m_pivotMotor.getPosition().getValueAsDouble()) < IntakeConstants.kGoalTolerance;
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

  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }

  public Trigger gotNote(){
    return new Trigger(this::hasNote);
  }

  @Override
  public void periodic() {

    SmartDashboard.putBoolean("IntakeAtGoal", isAtGoal());
    SmartDashboard.putNumber("IntakeDistToGoal", m_request.Position - m_pivotMotor.getPosition().getValueAsDouble());

    SmartDashboard.putBoolean("IntakeHasNote", hasNote());
    SmartDashboard.putNumber("IntakeSensor", m_distanceSensor.getAverageValue());
    SmartDashboard.putNumber("Intake position", m_pivotMotor.getPosition().getValueAsDouble());
  }
}