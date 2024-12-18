// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
  private final TalonFX m_pivotMotor = new TalonFX(IntakeConstants.kPivotMotorID);
  private final TalonFX m_rollersMotor = new TalonFX(IntakeConstants.kRollersMotorID);

  private final DigitalInput m_limitSwitch = new DigitalInput(IntakeConstants.kLimitSiwtchID);
  private final DigitalInput m_beamBreak = new DigitalInput(IntakeConstants.kBeamBreakID);
  private final Debouncer m_debouncer = new Debouncer(IntakeConstants.kDebounceTime, DebounceType.kFalling);

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine routine = new SysIdRoutine(            
    new SysIdRoutine.Config(
      null,
      Volts.of(4),
      null,
      (state) -> SignalLogger.writeString("state", state.toString())), new Mechanism((voltageMeasure)->m_pivotMotor.setControl(m_voltReq.withOutput(voltageMeasure.in(Volts))), null, this));

    
  public Intake() {
    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration()
      .withSlot0(IntakeConstants.kPivotGains)
      .withMotionMagic(IntakeConstants.kProfileConfigs);
    FeedbackConfigs feedbackConfigs = pivotConfigs.Feedback;
    feedbackConfigs.withSensorToMechanismRatio(IntakeConstants.kMotorToPivotRatio);
    
    m_pivotMotor.getConfigurator().apply(pivotConfigs);

    m_rollersMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(14).withStatorCurrentLimitEnable(true));
    
    BaseStatusSignal.setUpdateFrequencyForAll(100,
    m_pivotMotor.getPosition(),
    m_pivotMotor.getVelocity(),
    m_pivotMotor.getMotorVoltage(),
    m_pivotMotor.getTorqueCurrent(),
    m_rollersMotor.getTorqueCurrent());
    
    m_pivotMotor.optimizeBusUtilization(10, 0.05);
    m_rollersMotor.optimizeBusUtilization(10, 0.05);

    SmartDashboard.putData("Intake", this);
  }


  public void setPivotGoal(double position){
    m_pivotMotor.setControl(m_request.withPosition(position));
  }
  public Command pivotCtrCmd(double position){
    return runOnce(()->setPivotGoal(position));
  }
  public Command zeroIntake(){
    return runEnd(()->{m_pivotMotor.setControl(m_voltReq.withOutput(-2));m_rollersMotor.set(0);}, ()->{m_pivotMotor.setPosition(0);setPivotGoal(0);}).until(this::limitSwitch);
  }
  public boolean isAtGoal(){
    return Math.abs(m_request.Position - m_pivotMotor.getPosition().getValueAsDouble()) < IntakeConstants.kGoalTolerance;
  }


  public Command rollerCtrlCmd(double dutyCycle) {
    return runOnce(() -> m_rollersMotor.set(dutyCycle)); 
  }


  public boolean limitSwitch(){
    return m_limitSwitch.get();
  }
  public boolean hasNote(){
    return !m_debouncer.calculate(m_beamBreak.get());
  }
  public Trigger gotNote(){
    return new Trigger(this::hasNote);
  }


  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }
  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("IntakeAtGoal", isAtGoal());
    SmartDashboard.putBoolean("intake switch", limitSwitch());
    SmartDashboard.putBoolean("IntakeHasNote", hasNote());
  }
}