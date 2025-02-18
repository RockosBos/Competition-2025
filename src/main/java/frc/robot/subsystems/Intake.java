// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.LaserCan;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  //Motor Definitions
  private SparkMax IntakeIn = new SparkMax(Constants.ID_INTAKE_ROLLER, MotorType.kBrushless);
  private SparkMax IntakeRotate = new SparkMax(Constants.ID_INTAKE_ROTATE, MotorType.kBrushless);

  //PID Controller and Encoder Definitions
  private SparkClosedLoopController BetterLoppyDoopy = IntakeRotate.getClosedLoopController();
  private AbsoluteEncoder intakeAbsEncoder = IntakeRotate.getAbsoluteEncoder();

  //Configuration Definitions
  private SparkMaxConfig intakeInConfig = new SparkMaxConfig();
  private SparkMaxConfig intakeRotateConfig = new SparkMaxConfig();
  private AbsoluteEncoderConfig IAEC = new AbsoluteEncoderConfig();

  private LaserCan leftyLazy = new LaserCan(Constants.LASERCAN_INTAKE_LEFT);
  private LaserCan rightyLazy = new LaserCan(Constants.LASERCAN_INTAKE_RIGHT); 

  //Other Variables
  private double voltage = 0.0, targetPosition = Constants.INTAKE_ROTATE_HANDOFF_POS, intakeRotateTargetErr = 0.0;
  private boolean errFlag = false;

  //Logging variables
  private DoubleLogEntry intakeInAmpLog, intakeInVoltageLog;
  private DoubleLogEntry intakeRotateAmpLog, intakeRotateVoltageLog, intakeRotateTargetPositionLog, intakeRotateAbsCurrentPositionLog, intakeRotateErrLog;
  private BooleanLogEntry intakeRotateInPosLog;

  /** Creates a new Intake. */
  public Intake() {

    //Set Up Configurations
    intakeInConfig.inverted(false);
    intakeInConfig.idleMode(IdleMode.kCoast);
    intakeInConfig.smartCurrentLimit(20);
    
    intakeRotateConfig.inverted(false);
    intakeRotateConfig.idleMode(IdleMode.kBrake);
    intakeRotateConfig.smartCurrentLimit(20);

    IAEC.zeroOffset(Constants.OFFSET_INTAKE_ROTATE_ABS);
    intakeRotateConfig.absoluteEncoder.apply(IAEC);
    intakeRotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(Constants.P_INTAKE_ROTATE)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(Constants.MIN_OUTPUT_INTAKE_ROTATE, Constants.MAX_OUTPUT_INTAKE_ROTATE);

   //Apply Configurations
    IntakeRotate.configure(intakeRotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    IntakeIn.configure(intakeInConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //logging
    DataLog log = DataLogManager.getLog();

    intakeInAmpLog = new DoubleLogEntry(log, "/U/Intake/intakeInAmps");
    intakeInVoltageLog = new DoubleLogEntry(log, "/U/Intake/intakeInVoltage");

    intakeRotateAmpLog = new DoubleLogEntry(log, "/U/Intake/intakeRotateAmps");
    intakeRotateVoltageLog = new DoubleLogEntry(log, "/U/Intake/intakeRotateVoltage");
    intakeRotateTargetPositionLog = new DoubleLogEntry(log, "/U/Intake/intakeRotateTargetPosition");
    intakeRotateAbsCurrentPositionLog = new DoubleLogEntry(log, "/U/Intake/intakeRotateAbsCurrentPosition");
    intakeRotateErrLog = new DoubleLogEntry(log, "/U/Intake/intakeRotateErrLog");

  }

    /**
     * Set voltage that will be used for the Intake Roller Motor
     * 
     * @param voltage The voltage to provide for the motor
     */
  public void setMotorVoltage(double voltage){
    this.voltage = voltage;
  }

    /**
     * Set Target Position for the Intake Arm, measured in Rotations via an Absolute Encoder
     * 
     * @param targetPosition The target position for the intake Arm in Rotations
     */
  public void setTargetPostion(double targetPosition){
    this.targetPosition = targetPosition;
  }

    /**
     * Returns if the error flag is currently set
     */
  public boolean inErrState(){
    return errFlag;
  }

    /**
     * Returns if the Intake Arm is in position based on a threshold set in the constants file
     */
  public boolean inPosition(){
    if(Constants.THRESHOLD_INTAKE_ROTATE_POS > intakeRotateTargetErr){
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {

    //Calculate the distance between current location and target location in rotations
    intakeRotateTargetErr = Math.abs(targetPosition - intakeAbsEncoder.getPosition());

    //Set Intake Roller Voltage
    IntakeIn.setVoltage(voltage);

    //Set Closed Loop Controller for Intake Rotate Arm
    BetterLoppyDoopy.setReference(targetPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    //Logging

    intakeInAmpLog.append(IntakeIn.getOutputCurrent());
    intakeInVoltageLog.append(IntakeIn.getAppliedOutput());

    intakeRotateAmpLog.append(IntakeRotate.getOutputCurrent());
    intakeRotateVoltageLog.append(IntakeRotate.getAppliedOutput());
    intakeRotateTargetPositionLog.append(targetPosition);
    intakeRotateAbsCurrentPositionLog.append(intakeAbsEncoder.getPosition());
    intakeRotateErrLog.append(intakeRotateTargetErr);
    //intakeRotateInPosLog.append(this.inPosition());

    SmartDashboard.putNumber("IntakeRotateAbs", intakeAbsEncoder.getPosition());

  }
}
