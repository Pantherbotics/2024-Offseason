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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_leftClimber = new TalonFX(ClimberConstants.kLeftMotorID);
  private final TalonFX m_rightClimber = new TalonFX(ClimberConstants.kRightMotorID);

  private final DigitalInput m_leftSwitch = new DigitalInput(ClimberConstants.kLeftSwitchID);
  private final DigitalInput m_rightSwitch = new DigitalInput(ClimberConstants.kRightSwitchID);

  public Climber() {
    BaseStatusSignal.setUpdateFrequencyForAll(25, 
    m_leftClimber.getTorqueCurrent(),
    m_leftClimber.getTorqueCurrent());
    m_leftClimber.optimizeBusUtilization(10,0.05);
    m_rightClimber.optimizeBusUtilization(10,0.05);

    m_leftClimber.setNeutralMode(NeutralModeValue.Brake);
    m_rightClimber.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setLeft(double speed){
    m_leftClimber.set(speed);
  }

  public void setRight(double speed){
    m_rightClimber.set(speed);
  }

  public Command ctrlLeftCmd(double dutyCycle){
    return runOnce(()->setLeft(dutyCycle));
  }

  public Command ctrlRightCmd(double dutyCycle){
    return runOnce(()->setRight(-dutyCycle));
  }


  public boolean leftSwitch(){
    return m_leftSwitch.get();
  }

  public boolean rightSwitch(){
    return m_rightSwitch.get();
  }

  public Command climbUntilSwitches(){
    return Commands.parallel(
      ctrlLeftCmd(1).until(this::leftSwitch).andThen(ctrlLeftCmd(0)),
      ctrlRightCmd(1).until(this::rightSwitch).andThen(ctrlRightCmd(0)).asProxy()
    );
  } 

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climb left switch", leftSwitch());
    SmartDashboard.putBoolean("climb right switch", rightSwitch());

  }

}
