// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Score extends SubsystemBase {

private SparkMax agitator = new SparkMax(Constants.ID_SCORE_AGITATE, MotorType.kBrushless);

private SparkMax Claw = new SparkMax(Constants.ID_SCORE_CLAW, MotorType.kBrushless);
private AbsoluteEncoder ClawAbs = Claw.getAbsoluteEncoder();
private AbsoluteEncoderConfig ClawAbsConfig = new AbsoluteEncoderConfig();
private SparkClosedLoopController ClawLoopy = Claw.getClosedLoopController();
private SparkMaxConfig ClawConfig = new SparkMaxConfig();

private SparkMax pivot = new SparkMax(Constants.ID_SCORE_PIVOT, MotorType.kBrushless);
private AbsoluteEncoder pivotAbs = pivot.getAbsoluteEncoder();
private AbsoluteEncoderConfig pivotAbsConfig = new AbsoluteEncoderConfig();
private SparkClosedLoopController pivotLoopy = pivot.getClosedLoopController();
private SparkMaxConfig pivotConfig = new SparkMaxConfig();

private SparkMax rotate = new SparkMax(Constants.ID_SCORE_ROTATE, MotorType.kBrushless);
private AbsoluteEncoder rotateAbs = rotate.getAbsoluteEncoder();
private AbsoluteEncoderConfig rotateAbsConfig = new AbsoluteEncoderConfig();
private SparkClosedLoopController rotateLoopy = rotate.getClosedLoopController();
private SparkMaxConfig rotateConfig = new SparkMaxConfig();

private double rotateTargetPostion = 0.0, pivotTargetPostion = 0.0, clawTargetPostion = 0.0;
private double agitatorVoltage = 0.0;

DataLog log = DataLogManager.getLog();
private DoubleLogEntry clawTargetPositionLog, clawCurrentPositionLog;
private DoubleLogEntry pivotTargetPositionLog, pivotCurrentPositionLog;
private DoubleLogEntry rotateTargetPositionLog, rotateCurrentPositionLog;

  /** Creates a new Score. */
  public Score() {
    
      ClawAbsConfig.zeroOffset(Constants.OFFSET_SCORE_PIVOT_ABS);
      ClawConfig.absoluteEncoder.apply(ClawAbsConfig);
      ClawConfig.inverted(Constants.INVERT_SCORE_CLAW);
      ClawConfig.idleMode(Constants.IDLEMODE_SCORE_CLAW);
      ClawConfig.closedLoopRampRate(Constants.RAMPRATE_SCORE_CLAW);
      ClawConfig.smartCurrentLimit(Constants.CURRENTLIMIT_SCORE_CLAW);
      ClawConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.P_SCORE_CLAW)
      .i(0)
      .d(0)
      .velocityFF(0)
      .outputRange(Constants.MIN_OUTPUT_SCORE_CLAW, Constants.MAX_OUTPUT_SCORE_CLAW);
    
      Claw.configure(ClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      
      pivotAbsConfig.zeroOffset(Constants.OFFSET_SCORE_PIVOT_ABS);
      pivotConfig.absoluteEncoder.apply(pivotAbsConfig);
      pivotConfig.inverted(Constants.INVERT_SCORE_PIVOT);
      pivotConfig.idleMode(Constants.IDLEMODE_SCORE_PIVOT);
      pivotConfig.closedLoopRampRate(Constants.RAMPRATE_SCORE_PIVOT);
      pivotConfig.smartCurrentLimit(Constants.CURRENTLIMIT_SCORE_PIVOT);
      pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.P_SCORE_PIVOT)
      .i(0)
      .d(0)
      .velocityFF(0)
      .outputRange(Constants.MIN_OUTPUT_SCORE_PIVOT, Constants.MAX_OUTPUT_SCORE_PIVOT);
    
      pivot.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      rotateAbsConfig.zeroOffset(Constants.OFFSET_SCORE_ROTATE_ABS);
      rotateConfig.absoluteEncoder.apply(rotateAbsConfig);
      rotateConfig.inverted(Constants.INVERT_SCORE_ROTATE);
      rotateConfig.idleMode(Constants.IDLEMODE_SCORE_ROTATE);
      rotateConfig.closedLoopRampRate(Constants.RAMPRATE_SCORE_ROTATE);
      rotateConfig.smartCurrentLimit(Constants.CURRENTLIMIT_SCORE_ROTATE);
      rotateConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
      .p(Constants.P_SCORE_ROTATE)
      .i(0)
      .d(0)
      .velocityFF(0)
      .outputRange(Constants.MIN_OUTPUT_SCORE_ROTATE, Constants.MAX_OUTPUT_SCORE_ROTATE);
      
      rotate.configure(rotateConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      //logging

      clawTargetPositionLog = new DoubleLogEntry(log, "/U/Score/clawTargetPosition");
      clawCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/clawCurrentPosition");

      pivotTargetPositionLog = new DoubleLogEntry(log, "/U/Score/pivotTargetPosition");
      pivotCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/pivotCurrentPosition");

      rotateTargetPositionLog = new DoubleLogEntry(log, "/U/Score/rotateTargetPosition");
      rotateCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/rotateCurrentPosition");

  }

  public void setRotateTargetPostion(double targetPostionRotate){
    this.rotateTargetPostion = targetPostionRotate;
  }

  public void setPivotTargetPostion(double targetPostionPivot){
    this.pivotTargetPostion = targetPostionPivot;
  }

  public void setClawTargetPostion(double targetPostionClaw){
    this.clawTargetPostion = targetPostionClaw;
  }
    /**
   * Set Claw Target Position, There should be an Open and Closed Position.
   * 
   * @param agitatorVoltage The Requested voltage to set Agitator voltage to.
   * 
   */
  public void setAgitatorRollerVoltage(double agitatorVoltage){
    this.agitatorVoltage = agitatorVoltage;
  }

    /**
   * Set Arm Rotate Target Position, There should be three states: Left, Right, and Center
   * 
   * @param rotateTargetPosition The Requested Set Position on the absolute encoder for the closed loop
   * controller to navigate to.
   */
  public boolean atRotateTarget(){
    if(Constants.THRESHOLD_SCORE_ROTATE_POS > rotateTargetPostion){
       return true;
      }
      return false;
    }

    /**
   * Set Arm Pivot Target Position, There should be a high state and low state for each direction.
   * 
   * @param pivotTargetPosition The Requested Set Position on the absolute encoder for the closed loop
   * controller to navigate to.
   */
  public boolean atPivotTarget(){
    if (Constants.THRESHOLD_SCORE_PIVOT_POS > pivotTargetPostion) {
      return true;
    }
      return false;
  }

  
  /**
   * Set Claw Target Position, There should be an Open and Closed Position.
   * 
   * @param clawTargetPosition The Requested Set Position on the absolute encoder for the closed loop
   * controller to navigate to.
   */

   public boolean atClawTarget(){
    if (Constants.THRESHOLD_SCORE_CLAW_POS > clawTargetPostion) {
      return true;
    }
      return false;
  }

  /**
  * Returns if the Intake Arm is in position based on a threshold set in the constants file
  */
  public boolean inPosition(){
    if (this.atRotateTarget() && this.atPivotTarget() && this.atClawTarget()) {
      return true;
    }
    return false;
  }

  @Override
  public void periodic() {
    
    
    // Logging
    clawCurrentPositionLog.append(ClawAbs.getPosition());
    clawTargetPositionLog.append(clawTargetPostion);

    pivotCurrentPositionLog.append(pivotAbs.getPosition());
    pivotTargetPositionLog.append(pivotTargetPostion);

    rotateCurrentPositionLog.append(rotateAbs.getPosition());
    rotateTargetPositionLog.append(rotateTargetPostion);
  }
}
