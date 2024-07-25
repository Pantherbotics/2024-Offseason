// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Shooter extends SubsystemBase {

  private final TalonFX m_pivotMotor;

  private final DutyCycleEncoder m_encoder;

  private final AnalogInput m_topSensor;
  private final AnalogInput m_sideSensor;

  private final TalonFX m_leftFlywheel;
  private final TalonFX m_rightFlywheel;
  private final BangBangController m_leftController;
  private final BangBangController m_rightController;


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

    m_leftFlywheel = new TalonFX(ShooterConstants.kLeftFlywheelMotorID);
    m_rightFlywheel = new TalonFX(ShooterConstants.kRightFlywheelMotorID);
    m_leftFlywheel.setNeutralMode(NeutralModeValue.Coast);
    m_rightFlywheel.setNeutralMode(NeutralModeValue.Coast);

    m_topSensor = new AnalogInput(ShooterConstants.kTopSensorID);
    m_sideSensor = new AnalogInput(ShooterConstants.kSideSensorID);

    m_topSensor.setAverageBits(4);
    m_sideSensor.setAverageBits(4);

    m_leftController = new BangBangController(ShooterConstants.kBangBangTolerance);
    m_rightController = new BangBangController(ShooterConstants.kBangBangTolerance);

    handoffPosition();


    SmartDashboard.putNumber("ShooterTopSensor", m_topSensor.getAverageValue());
    SmartDashboard.putBoolean("ShooterHasNote", noteSeated());

    SmartDashboard.putBoolean("ShooterAtGoal", isAtGoal());
    SmartDashboard.putNumber("ShooterDistToGoal", this.goal.position - m_encoder.get());
  }

  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setVoltage(voltageMeasure.magnitude());
  }

  public void setPivotGoal(double goal){
    this.goal.position = goal;
  }

  public boolean isAtGoal(){
    return MathUtil.isNear(this.goal.position, m_pivotMotor.get(), ShooterConstants.kPivotTolerance);
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

  public void setRollers(double speed){
    // TODO: add rollers
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
    var nextSetpoint = profile.calculate(ShooterConstants.dt, currentSetpoint, goal);

    m_pivotMotor.setVoltage(
        feedforward.calculate(Units.rotationsToRadians(m_encoder.get()), Units.rotationsToRadians(m_pivotMotor.getVelocity().getValueAsDouble()), Units.rotationsToRadians(m_pivotMotor.getAcceleration().getValueAsDouble()))
            + controller.calculate(m_encoder.get(), currentSetpoint.position));

    currentSetpoint = nextSetpoint;

    m_leftFlywheel.set(
      m_leftController.calculate(m_leftFlywheel.getVelocity().getValueAsDouble())
    );

    m_rightFlywheel.set(
      -m_rightController.calculate(-m_rightFlywheel.getVelocity().getValueAsDouble())
    );

  }
}
