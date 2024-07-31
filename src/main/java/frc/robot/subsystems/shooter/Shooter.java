// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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

  private final CANSparkMax m_leftIntake;
  private final CANSparkMax m_rightIntake;

  private final TalonFX m_leftFlywheel;
  private final TalonFX m_rightFlywheel;
  private final BangBangController m_leftController;
  private final BangBangController m_rightController;
  
  private InterpolatingDoubleTreeMap shotTable;
  private InterpolatingDoubleTreeMap passTable;


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

    m_leftIntake = new CANSparkMax(ShooterConstants.kLeftIntakeMotorID, MotorType.kBrushless);
    m_rightIntake = new CANSparkMax(ShooterConstants.kRightIntakeMotorID, MotorType.kBrushless);
    m_leftIntake.setIdleMode(IdleMode.kBrake);
    m_rightIntake.setIdleMode(IdleMode.kBrake);
    m_leftIntake.follow(m_leftIntake, true);

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

    shotTable = new InterpolatingDoubleTreeMap();
    for(int i = 0; i < ShooterConstants.shotMatrix.length; i++){
        shotTable.put(ShooterConstants.shotMatrix[i][0], ShooterConstants.shotMatrix[i][1]);
    }

    passTable = new InterpolatingDoubleTreeMap();
    for(int i = 0; i < ShooterConstants.passMatrix.length; i++){
        passTable.put(ShooterConstants.passMatrix[i][0], ShooterConstants.passMatrix[i][1]);
    }

  
    setPivotGoal(ShooterConstants.kHandoffPosition);

    SmartDashboard.putData("shooter", this);
    SmartDashboard.putData("Shooter Controller", controller);

  }

  public InterpolatingDoubleTreeMap getShotTable(){
    return shotTable;
  }

  public InterpolatingDoubleTreeMap getPassTable(){
    return passTable;
  }
  
  public void pivotVoltage(Measure<Voltage> voltageMeasure){
    m_pivotMotor.setVoltage(voltageMeasure.magnitude());
  }

  public void setPivotGoal(double goal){
    this.goal.position = goal;
  }

  public boolean isAtGoal(){
    return MathUtil.isNear(this.goal.position, m_pivotMotor.getPosition().getValueAsDouble(), ShooterConstants.kPivotTolerance);
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

    SmartDashboard.putBoolean("ShooterAtGoal", isAtGoal());
    SmartDashboard.putNumber("ShooterDistToGoal", this.goal.position - m_encoder.get());
    SmartDashboard.putNumber("shooterEncoderPosition", m_encoder.getAbsolutePosition());

    var nextSetpoint = profile.calculate(ShooterConstants.dt, currentSetpoint, goal);
    var encoderValue = m_encoder.get();
    double feed = feedforward.calculate(Units.rotationsToRadians(encoderValue), Units.rotationsToRadians(m_pivotMotor.getVelocity().getValueAsDouble()), Units.rotationsToRadians(m_pivotMotor.getAcceleration().getValueAsDouble()));
    double control = controller.calculate(encoderValue, currentSetpoint.position);
    m_pivotMotor.setVoltage(feed + control);
    currentSetpoint = nextSetpoint;

    m_leftFlywheel.set(
      m_leftController.calculate(m_leftFlywheel.getVelocity().getValueAsDouble())
    );

    m_rightFlywheel.set(
      -m_rightController.calculate(-m_rightFlywheel.getVelocity().getValueAsDouble())
    );
  }
}
