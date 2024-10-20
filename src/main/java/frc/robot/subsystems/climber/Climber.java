// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private final TalonFX m_leftClimber = new TalonFX(ClimberConstants.kLeftMotorID);
  private final TalonFX m_rightClimber = new TalonFX(ClimberConstants.kRightMotorID);

  private final DigitalInput m_leftSwitch = new DigitalInput(ClimberConstants.kLeftSwitchID);
  private final DigitalInput m_rightSwitch = new DigitalInput(ClimberConstants.kRightSwitchID);

  private final PositionVoltage positionReq = new PositionVoltage(0, 0, false, 0, 0, true, false, false);

  public Climber() {
    BaseStatusSignal.setUpdateFrequencyForAll(25, 
    m_leftClimber.getTorqueCurrent(),
    m_leftClimber.getTorqueCurrent());
    m_leftClimber.optimizeBusUtilization(10,0.05);
    m_rightClimber.optimizeBusUtilization(10,0.05);

    m_rightClimber.setPosition(0);
    m_leftClimber.setPosition(0);
    m_leftClimber.setNeutralMode(NeutralModeValue.Brake);
    m_rightClimber.setNeutralMode(NeutralModeValue.Brake);

    //m_leftClimber.getConfigurator().apply(new Slot0Configs().withKP(1).withKI(0).withKG(0).withKS);
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

  public Command ctrClimbersCmd(double dutyCycle){
    return runOnce(()->{
      setLeft(-dutyCycle);
      setRight(dutyCycle);
    });
  }

  public Command unclimbCmd(){
    return runOnce(
      ()->{
        m_leftClimber.setControl(positionReq.withPosition(0));
        m_rightClimber.setControl(positionReq.withPosition(0));
      }
    );
  }


  public boolean leftSwitch(){
    return m_leftSwitch.get();
  }

  public boolean rightSwitch(){
    return m_rightSwitch.get();
  }


  @Override
  public void periodic() {
    SmartDashboard.putBoolean("climb left switch", leftSwitch());
    SmartDashboard.putBoolean("climb right switch", rightSwitch());

  }

}
