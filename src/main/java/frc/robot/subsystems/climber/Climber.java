// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_leftClimber;
  private final TalonFX m_rightClimber;

  private final DigitalInput m_leftSwitch;
  private final DigitalInput m_rightSwitch;

  public Climber() {
    m_leftClimber = new TalonFX(ClimberConstants.kLeftMotorID);
    m_rightClimber = new TalonFX(ClimberConstants.kRightMotorID);
    BaseStatusSignal.setUpdateFrequencyForAll(25, 
    m_leftClimber.getTorqueCurrent(),
    m_leftClimber.getTorqueCurrent());
    m_leftClimber.optimizeBusUtilization(10,0.05);
    m_rightClimber.optimizeBusUtilization(10,0.05);

    m_leftSwitch = new DigitalInput(ClimberConstants.kLeftSwitchID);
    m_rightSwitch = new DigitalInput(ClimberConstants.kRightSwitchID);

    m_leftClimber.setNeutralMode(NeutralModeValue.Brake);
    m_rightClimber.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setLeft(double speed){
    m_leftClimber.set(speed);
  }

  public void setRight(double speed){
    m_rightClimber.set(speed);
  }

  public boolean leftSwitch(){
    return m_leftSwitch.get();
  }

  public boolean rightSwitch(){
    return m_rightSwitch.get();
  }

  public Command climbUntilSwitches(){
    return runEnd(()->m_leftClimber.set(1), ()->m_leftClimber.set(0)).until(this::leftSwitch).asProxy().alongWith(runEnd(()->m_rightClimber.set(-1), ()->m_rightClimber.set(0)).until(this::rightSwitch));
  } 

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climb left switch", leftSwitch());
    SmartDashboard.putBoolean("climb right switch", rightSwitch());

  }

}
