// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Talon FX for turret motor base (kraken)
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

//wpi imports for SparkMax motors and encoders
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import com.revrobotics.spark.config.ClosedLoopConfig;
import edu.wpi.first.math.controller.ArmFeedforward; //to maintain angle against gravity 

//pid WPI
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

//values
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import static frc.robot.Constants.TurretConstants.*;

//tele
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//subsystems structure
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * Subsystem Overview
 * This subsystem controls the turret mechanism of the robot, allowing it to rotate to specific angles for targeting.
 * 
 * requirements:
 * - Kraken (base)
 * - Neo (angle)
 * - CanCoder (Base)
 * - throughbore
 * - laser
 * 
 * methods:
 * - RotateBase(double speed)
 * - ChangeAngle(double speed)
 * - laserIsOn(boolean state)
 * - GetEncoderAngleBase()
 * - GetEncoderAngleTurret()
 * 
 */
public class Turret extends SubsystemBase {

  //motors
  private TalonFX turretMotorBase;
  private TalonFXConfiguration turretMotorBaseConfig;

  private SparkMax turretMotorAngle;
  private SparkMaxConfig turretMotorAngleConfig;
  
  //encoders
  private CANcoder turretEncoderBase;
  private DutyCycleEncoder turretEncoderAngle;

  //PID Controllers
  private Slot0Configs turretBasePIDConfig; //kraken PID
  private ClosedLoopConfig turretAnglePIDConfig; //neo PID 
  private PIDController turretAnglePID; //neo PID
  
  //motor controller assistance
  private VoltageOut turretBaseVoltageControl;
  private VoltageOut turretAngleVoltageControl;
  private ArmFeedforward turretAngleFeedforward;

  final private PositionVoltage turretBasePositionControl;
  // final private PositionVoltage turretAnglePositionControl;
  final private DutyCycleOut turreAngleDutyCycleControl;
  // final private DutyCycleOut turretAngleDutyCycleControl;

  //trapezoid profiles
  final private TrapezoidProfile m_TrapezoidProfileBase;
  private TrapezoidProfile.State m_TrapezoidProfileBaseGoal;//desired state
  private TrapezoidProfile.State m_TrapezoidProfileBaseSetpoint; //current state

  // final private TrapezoidProfile m_TrapezoidProfileAngle;
  // final private TrapezoidProfile.State m_TrapezoidProfileAngleGoal;//desired state
  // final private TrapezoidProfile.State m_TrapezoidProfileAngleSetpoint; //current state

  //laser
  // I don't know what goes here. fix later

  //other values
  private boolean laserState;


  

  public Turret() {

    //base construction
    turretMotorBase = new TalonFX(kTurretEncoderPortBase);

    turretMotorBaseConfig = new TalonFXConfiguration();
    turretMotorBaseConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    turretMotorBase.setNeutralMode(NeutralModeValue.Brake);
    turretMotorBase.getConfigurator().apply(turretMotorBaseConfig);

      //slot 0 PID - kraken
      turretBasePIDConfig = new Slot0Configs();
      turretBasePIDConfig.kP = kTurretBaseP;
      turretBasePIDConfig.kI = kTurretBaseI;
      turretBasePIDConfig.kD = kTurretBaseD;
      
      turretMotorBase.getConfigurator().apply(turretBasePIDConfig);

      turretBasePositionControl = new PositionVoltage(0).withEnableFOC(true);
      

      //trapezopid profile - base
      m_TrapezoidProfileBase = new TrapezoidProfile(new TrapezoidProfile.Constraints(kBaseMaxVoltage, kBaseMaxAcceleration));
      m_TrapezoidProfileBaseGoal = new TrapezoidProfile.State(0,0);
      m_TrapezoidProfileBaseSetpoint = new TrapezoidProfile.State();

    //angle construction
    turretMotorAngle = new SparkMax(kTurretMotorPortAngle, MotorType.kBrushless);

    turretMotorAngleConfig = new SparkMaxConfig();
    turretMotorAngleConfig.idleMode(IdleMode.kBrake);
    turretMotorAngleConfig.inverted(false);
    turretMotorAngle.configure(turretMotorAngleConfig, null, null);

      //closed loop PID - neo
      turretAnglePIDConfig = new ClosedLoopConfig();
      turretAnglePIDConfig.pid(kTurretBaseP, kTurretBaseI, kTurretBaseD);
      turretMotorAngle.configure(turretMotorAngleConfig, null, null);

      turretAngleFeedforward = new ArmFeedforward(0, 0,0, 0); //tune later
      turreAngleDutyCycleControl = new DutyCycleOut(0).withEnableFOC(true);
      turretAnglePID = new PIDController(kTurretAngleP, kTurretAngleI, kTurretAngleD);

    
    //encoders  
    turretEncoderBase = new CANcoder(kTurretEncoderPortBase);
    turretEncoderAngle = new DutyCycleEncoder(kTurretEncoderPortAngle, kTurretAngleEncoderFullRotationValue, kTurretAngleEncoderZeroPosition);

    //everything else
    laserState = false;
  }


  //------------------------//
  //-----Base-Functions-----//
  //------------------------//

  public void manualRotateBase(double speed) {
    speed = speed * kSpeedBaseModifier;
    turretMotorBase.set(speed);
  }

  public void rotateBaseToAngle(double angle) {
    //set goal
    m_TrapezoidProfileBaseGoal.position = angle;
    m_TrapezoidProfileBaseSetpoint = m_TrapezoidProfileBase.calculate(kTrapezoidTimeIntervale, m_TrapezoidProfileBaseSetpoint, m_TrapezoidProfileBaseGoal);

    turretBasePositionControl.Position = m_TrapezoidProfileBaseSetpoint.position;
    turretBasePositionControl.Velocity = m_TrapezoidProfileBaseSetpoint.velocity;
    turretMotorBase.setControl(turreAngleDutyCycleControl);
 
  }

  public void stopBaseMotor() {
    turretMotorBase.set(0);
  }

  //------------------------//
  //-----Angle-Functions----//
  //------------------------//

  public void manualChangeAngle(double speed) {
    speed = speed * kSpeedAngleModifier;
    turretMotorAngle.set(speed);
  }

  public void changeAngleToPosition(double position) {
    turretMotorAngle.set(turretAnglePID.calculate(turretEncoderAngle.get(), position));
  }

  //github im begging you pleaseeeee
  
  public void stopAngleMotor() {
    turretMotorAngle.set(0);
  }
  

  //getters
  public double getEncoderAngleBase() {
    return turretEncoderBase.getPosition().getValueAsDouble();
  }

  public boolean isInRotationLimitsBase() {
    boolean inBounds = getEncoderAngleBase() >= kTurretBaseMinAngle && getEncoderAngleBase() <= kTurretBaseMaxAngle;
    return inBounds;
  }

  public double getEncoderAngleTurret() {
    return turretEncoderAngle.get();
  }

  public boolean isInAngleLimitsAngle() {
    boolean inBounds = getEncoderAngleTurret() >= kTurretMinAngle && getEncoderAngleTurret() <= kTurretMaxAngle;
    return inBounds;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
