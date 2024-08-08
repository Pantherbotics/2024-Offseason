// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.BangBangController;
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

  private final TalonFX m_pivotMotor;

  private final CANcoder m_encoder;

  private final AnalogInput m_topSensor;
  private final AnalogInput m_sideSensor;

  private final CANSparkMax m_leftIntake;
  private final CANSparkMax m_rightIntake;

  private final TalonFX m_leftFlywheel;
  private final TalonFX m_rightFlywheel;
  private final BangBangController m_leftController;
  private final BangBangController m_rightController;
  
  private final InterpolatingDoubleTreeMap shotTable;
  private final InterpolatingDoubleTreeMap passTable;

  private final MotionMagicVoltage m_request;
  private final VoltageOut m_voltReq = new VoltageOut(0.0);

  private final SysIdRoutine routine = new SysIdRoutine(new Config(
    null, Volts.of(4),
    null,
    (state) -> SignalLogger.writeString("state", state.toString())),
     new Mechanism(this::pivotVoltage, null, this));

     
  public Shooter() {
    //Rollers setup
    m_leftIntake = new CANSparkMax(ShooterConstants.kLeftIntakeMotorID, MotorType.kBrushless);
    m_rightIntake = new CANSparkMax(ShooterConstants.kRightIntakeMotorID, MotorType.kBrushless);
    m_leftIntake.setIdleMode(IdleMode.kBrake);
    m_rightIntake.setIdleMode(IdleMode.kBrake);
    m_rightIntake.follow(m_leftIntake, true);

    
    //Flywheel setup
    m_leftFlywheel = new TalonFX(ShooterConstants.kLeftFlywheelMotorID);
    m_rightFlywheel = new TalonFX(ShooterConstants.kRightFlywheelMotorID);
    m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
    m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);
    m_leftController = new BangBangController(ShooterConstants.kBangBangTolerance);
    m_rightController = new BangBangController(ShooterConstants.kBangBangTolerance);
    //Sensor setup
    m_topSensor = new AnalogInput(ShooterConstants.kTopSensorID);
    m_sideSensor = new AnalogInput(ShooterConstants.kSideSensorID);
    m_topSensor.setAverageBits(4);
    m_sideSensor.setAverageBits(4);

    //Pivot setup
    m_pivotMotor = new TalonFX(ShooterConstants.kPivotMotorID);
    m_encoder = new CANcoder(ShooterConstants.kEncoderID);

    m_encoder.getConfigurator().apply(new MagnetSensorConfigs().withMagnetOffset(ShooterConstants.kEncoderOffset));

    TalonFXConfiguration pivotConfigs = new TalonFXConfiguration()
    .withSlot0(ShooterConstants.kPivotGains)
    .withMotionMagic(ShooterConstants.kProfileConfigs)
    .withFeedback(ShooterConstants.kFeedbackConfigs);
  
    m_pivotMotor.getConfigurator().apply(pivotConfigs);

    BaseStatusSignal.setUpdateFrequencyForAll(100,
      m_pivotMotor.getPosition(),
      m_pivotMotor.getVelocity(),
      m_pivotMotor.getMotorVoltage(),
      m_encoder.getPosition(),
      m_encoder.getVelocity(),
      m_leftFlywheel.getVelocity(),
      m_rightFlywheel.getVelocity());
    m_pivotMotor.optimizeBusUtilization(10, 0.05);
    m_encoder.optimizeBusUtilization(10, 0.05);
    m_leftFlywheel.optimizeBusUtilization();
    m_rightFlywheel.optimizeBusUtilization();

    
    m_request = new MotionMagicVoltage(0);

    shotTable = new InterpolatingDoubleTreeMap();
    for(int i = 0; i < ShooterConstants.shotMatrix.length; i++){
        shotTable.put(ShooterConstants.shotMatrix[i][0], ShooterConstants.shotMatrix[i][1]);
    }

    passTable = new InterpolatingDoubleTreeMap();
    for(int i = 0; i < ShooterConstants.passMatrix.length; i++){
        passTable.put(ShooterConstants.passMatrix[i][0], ShooterConstants.passMatrix[i][1]);
    }

    SmartDashboard.putData("shooter", this);
  }



  public InterpolatingDoubleTreeMap getShotTable(){
    return shotTable;
  }

  public InterpolatingDoubleTreeMap getPassTable(){
    return passTable;
  }
  
  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setControl(m_voltReq.withOutput(voltageMeasure.in(Volts)));
  }

  public void setPivotGoal(double goal){
    m_pivotMotor.setControl(m_request.withPosition(goal));
  }

  public boolean isAtGoal(){
    return Math.abs(m_request.Position - m_pivotMotor.getPosition().getValueAsDouble()) < ShooterConstants.kPivotTolerance;
  }

  public void setFlywheelSpeed(double goal){
    m_leftController.setSetpoint(goal);
    m_rightController.setSetpoint(goal);
  }

  public boolean flywheelsAtSetpoint(){
    return m_leftController.atSetpoint() && m_rightController.atSetpoint();
  }


  public boolean noteSeated(){
    return m_topSensor.getAverageValue() > ShooterConstants.kTopSensorThreshold;
  }

  public boolean sideSensor(){
    return m_sideSensor.getAverageValue() > ShooterConstants.kSideSensorThreshold;
  }

  public void setRollers(double speed){
    m_leftIntake.set(speed);
  }

  public Command rollersIn(){
    return runOnce(()->setRollers(ShooterConstants.kRollersInSpeed));
  }

  public Command rollersOut(){
    return runOnce(()->setRollers(ShooterConstants.kRollersOutSpeed));
  }

  public Command rollersStop(){
    return runOnce(()->setRollers(0));
  }

  public Command ampPosition(){
    return runOnce(()->setPivotGoal(ShooterConstants.kAmpPosition));
  }

  public Command handoffPosition(){
    return runOnce(()->setPivotGoal(ShooterConstants.kHandoffPosition));
  }
  
  public Command sysIdDynamicCommand(Direction direction){
    return routine.dynamic(direction);
  }

  public Command sysIdQuasistaticCommand(Direction direction){
    return routine.quasistatic(direction);
  }
  

  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShooterTopSensor", m_topSensor.getAverageValue());
    SmartDashboard.putBoolean("ShooterHasNote", noteSeated());
    SmartDashboard.putNumber("ShooterSideSensor", m_sideSensor.getAverageValue());
    SmartDashboard.putBoolean("sideSeesNote", sideSensor());

    SmartDashboard.putBoolean("ShooterAtGoal", isAtGoal());
    SmartDashboard.putNumber("shooterEncoderAbsPosition", m_encoder.getAbsolutePosition().getValueAsDouble());
    SmartDashboard.putNumber("shooterEncoderPosition", m_encoder.getPosition().getValueAsDouble());

    SmartDashboard.putNumber("shooter motor", m_pivotMotor.getPosition().getValueAsDouble());



    SmartDashboard.putNumber("left flywheel", m_leftFlywheel.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("right flywheel", m_rightFlywheel.getVelocity().getValueAsDouble());

    m_leftFlywheel.set(
      -m_leftController.calculate(-Math.min(m_leftFlywheel.getVelocity().getValueAsDouble(), 0))
    );

    m_rightFlywheel.set(
      m_rightController.calculate(Math.max(m_rightFlywheel.getVelocity().getValueAsDouble(), 0))
    );
    
  }



  public Command speedUp() {
    return runOnce(()->setFlywheelSpeed(100));
  }
}
