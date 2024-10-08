// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
public class Shooter extends SubsystemBase {

  private final TalonFX m_pivotMotor = new TalonFX(ShooterConstants.kPivotMotorID);

  private final CANcoder m_encoder = new CANcoder(ShooterConstants.kEncoderID);

  private final AnalogInput m_topSensor = new AnalogInput(ShooterConstants.kTopSensorID);
  private final AnalogInput m_sideSensor = new AnalogInput(ShooterConstants.kSideSensorID);

  private final CANSparkMax m_leftIntake = new CANSparkMax(ShooterConstants.kLeftIntakeMotorID, MotorType.kBrushless);
  private final CANSparkMax m_rightIntake = new CANSparkMax(ShooterConstants.kRightIntakeMotorID, MotorType.kBrushless);

  private final TalonFX m_leftFlywheel = new TalonFX(ShooterConstants.kLeftFlywheelMotorID);
  private final TalonFX m_rightFlywheel = new TalonFX(ShooterConstants.kRightFlywheelMotorID);
  
  private final InterpolatingDoubleTreeMap shotTable = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap passTable = new InterpolatingDoubleTreeMap();

  private final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
  private final VelocityVoltage m_velocityRequest = new VelocityVoltage(0);
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine routine = new SysIdRoutine(new Config(
    null, Volts.of(4),
    null,
    (state) -> SignalLogger.writeString("state", state.toString())),
     new Mechanism(this::pivotVoltage, null, this));


     
  public Shooter() {
    //Rollers setup
    m_leftIntake.setIdleMode(IdleMode.kBrake);
    m_rightIntake.setIdleMode(IdleMode.kBrake);
    m_rightIntake.follow(m_leftIntake, true);

    //Sensor setup
    m_topSensor.setAverageBits(4);
    m_sideSensor.setAverageBits(4);
    
    //Flywheel setup
    m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
    m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);
    m_leftFlywheel.getConfigurator().apply(new TalonFXConfiguration().withSlot0(ShooterConstants.kLeftFlywheelGains));
    m_rightFlywheel.getConfigurator().apply(new TalonFXConfiguration().withSlot0(ShooterConstants.kRightFlywheelGains));

    //Pivot setup
    m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(ShooterConstants.kEncoderOffset));

    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration()
    .withSlot0(ShooterConstants.kPivotGains)
    .withMotionMagic(ShooterConstants.kProfileConfigs)
    .withFeedback(ShooterConstants.kFeedbackConfigs);
  
    m_pivotMotor.getConfigurator().apply(pivotConfigs);
    m_pivotMotor.setNeutralMode(NeutralModeValue.Brake);

    BaseStatusSignal.setUpdateFrequencyForAll(100,
      m_pivotMotor.getPosition(),
      m_pivotMotor.getVelocity(),
      m_pivotMotor.getMotorVoltage(),
      m_pivotMotor.getTorqueCurrent(),
      m_encoder.getPosition(),
      m_encoder.getVelocity(),
      m_leftFlywheel.getVelocity(),
      m_leftFlywheel.getTorqueCurrent(),
      m_rightFlywheel.getVelocity(),
      m_rightFlywheel.getTorqueCurrent()
    );
    m_pivotMotor.optimizeBusUtilization(4, 0.05);
    m_encoder.optimizeBusUtilization(4, 0.05);
    m_leftFlywheel.optimizeBusUtilization();
    m_rightFlywheel.optimizeBusUtilization();

    for(int i = 0; i < ShooterConstants.shotMatrix.length; i++){
        shotTable.put(ShooterConstants.shotMatrix[i][0], ShooterConstants.shotMatrix[i][1]);
    }
    for(int i = 0; i < ShooterConstants.passMatrix.length; i++){
        passTable.put(ShooterConstants.passMatrix[i][0], ShooterConstants.passMatrix[i][1]);
    }
  }



  public InterpolatingDoubleTreeMap getShotTable(){
    return shotTable;
  }

  public InterpolatingDoubleTreeMap getPassTable(){
    return passTable;
  }
  

  public void setPivotGoal(double goal){
    m_pivotMotor.setControl(m_request.withPosition(goal));
  }

  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setControl(m_voltReq.withOutput(voltageMeasure.in(Volts)));
  }

  public Command pivotCtrlCmd(double position){
    return runOnce(()->setPivotGoal(position));
  }

  public double pivotAngle(){
    return m_pivotMotor.getPosition().getValueAsDouble();
  }

  public boolean isAtGoal(){
    return Math.abs(m_pivotMotor.getClosedLoopError().getValueAsDouble()) < ShooterConstants.kPivotTolerance;
  }




  public void setFlywheelSpeed(double goal){
    m_leftFlywheel.setControl(m_velocityRequest.withVelocity(goal));
    m_rightFlywheel.setControl(m_velocityRequest.withVelocity(goal));
  }

  public Command FlywheelCtrCmd(double velocity){
    return runOnce(()->setFlywheelSpeed(velocity));
  }

  public boolean flywheelsAtSetpoint(){
    return m_leftFlywheel.getClosedLoopError().getValueAsDouble() < ShooterConstants.kFlywheelTolerance && m_rightFlywheel.getClosedLoopError().getValueAsDouble() < ShooterConstants.kFlywheelTolerance;
  }

  public Command coastFlywheelsCmd() {
    return runOnce(()->{m_leftFlywheel.setControl(new CoastOut());m_rightFlywheel.setControl(new CoastOut());});
  }



  public boolean topSensor(){
    return m_topSensor.getAverageValue() > ShooterConstants.kTopSensorThreshold;
  }

  public boolean sideSensor(){
    return m_sideSensor.getAverageValue() > ShooterConstants.kSideSensorThreshold;
  }



  public void setRollers(double speed){
    m_leftIntake.set(speed);
  }

  public Command rollerCtrlCmd(double dutyCycle) {
    return runOnce(()->setRollers(dutyCycle));
  }


  
  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }
  

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("topSensorValue", m_topSensor.getAverageValue());
    SmartDashboard.putBoolean("topSensor", topSensor());
    SmartDashboard.putNumber("sideSensorValue", m_sideSensor.getAverageValue());
    SmartDashboard.putBoolean("sideSensor", sideSensor());

    SmartDashboard.putBoolean("ShooterAtGoal", isAtGoal());
    SmartDashboard.putNumber("shooterEncoderPosition", m_encoder.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("left flywheel", m_leftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right flywheel", m_rightFlywheel.getVelocity().getValueAsDouble());

  }




}
