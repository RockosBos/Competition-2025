// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ScoreState;

public class Score extends SubsystemBase {

private SparkMax agitator = new SparkMax(Constants.ID_SCORE_AGITATE, MotorType.kBrushless);
private SparkMaxConfig agitatorConfig = new SparkMaxConfig();

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

private double rotateTargetPostion = Constants.SCORE_ROTATE_CENTER_POS, pivotTargetPostion = Constants.SCORE_PIVOT_IN_POS, clawTargetPostion = Constants.SCORE_CLAW_OPEN_POS;
private double agitatorVoltage = 0.0;

private ScoreState scoreState = ScoreState.CENTER;

private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
private final NetworkTable table = inst.getTable("Score");
private final DoublePublisher scoreRotatePosPub = table.getDoubleTopic("Score").publish(),
                              scoreRotateSetpointPub = table.getDoubleTopic("Score").publish(),
                              scoreRotateAmpsPub = table.getDoubleTopic("Score").publish(), 
                              scorePivotPosPub = table.getDoubleTopic("Score").publish(),
                              scorePivotSetpointPub = table.getDoubleTopic("Score").publish(),
                              scorePivotAmpsPub = table.getDoubleTopic("Score").publish(),
                              scoreClawPosPub = table.getDoubleTopic("Score").publish(),
                              scoreClawSetpointPub = table.getDoubleTopic("Score").publish(),
                              scoreClawAmpsPub = table.getDoubleTopic("Score").publish();

DataLog log = DataLogManager.getLog();
private DoubleLogEntry clawTargetPositionLog, clawCurrentPositionLog, clawAmpsLog;
private DoubleLogEntry pivotTargetPositionLog, pivotCurrentPositionLog, pivotAmpsLog;
private DoubleLogEntry rotateTargetPositionLog, rotateCurrentPositionLog, rotateAmpsLog;

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
      pivotAbsConfig.inverted(Constants.INVERT_SCORE_PIVOT_ABS);
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

      agitatorConfig.inverted(Constants.INVERT_SCORE_AGITATOR);
      agitatorConfig.smartCurrentLimit(Constants.CURRENTLIMIT_SCORE_AGITATOR);
      agitatorConfig.idleMode(Constants.IDLEMODE_SCORE_AGITATOR);
      agitator.configure(agitatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      //logging

      clawTargetPositionLog = new DoubleLogEntry(log, "/U/Score/clawTargetPosition");
      clawCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/clawCurrentPosition");
      clawAmpsLog = new DoubleLogEntry(log, "/U/Score/ClawAmps");

      pivotTargetPositionLog = new DoubleLogEntry(log, "/U/Score/pivotTargetPosition");
      pivotCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/pivotCurrentPosition");
      pivotAmpsLog = new DoubleLogEntry(log, "/U/Score/pivotAmps");

      rotateTargetPositionLog = new DoubleLogEntry(log, "/U/Score/rotateTargetPosition");
      rotateCurrentPositionLog = new DoubleLogEntry(log, "/U/Score/rotateCurrentPosition");
      rotateAmpsLog = new DoubleLogEntry(log, "/U/Score/RotateAmps");

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

  public void setScoreState(ScoreState state){
    this.scoreState = state;
  }

  public ScoreState getScoreState(){
    return this.scoreState;
  }

    /**
   * Set Arm Rotate Target Position, There should be three states: Left, Right, and Center
   * 
   * @param rotateTargetPosition The Requested Set Position on the absolute encoder for the closed loop
   * controller to navigate to.
   */
  public boolean atRotateTarget(){
    if(Constants.THRESHOLD_SCORE_ROTATE_POS > Math.abs(rotateAbs.getPosition() - rotateTargetPostion)){
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
    if (Constants.THRESHOLD_SCORE_PIVOT_POS > Math.abs(pivotAbs.getPosition() - pivotTargetPostion)) {
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
    if (Constants.THRESHOLD_SCORE_CLAW_POS > Math.abs(ClawAbs.getPosition() - clawTargetPostion)) {
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
    
    // rotateLoopy.setReference(rotateTargetPostion, ControlType.kPosition);
    // pivotLoopy.setReference(pivotTargetPostion, ControlType.kPosition);
    // ClawLoopy.setReference(clawTargetPostion, ControlType.kPosition);

    // agitator.setVoltage(agitatorVoltage);
    
    // Logging
    clawCurrentPositionLog.append(ClawAbs.getPosition());
    clawTargetPositionLog.append(clawTargetPostion);
    clawAmpsLog.append(Claw.getOutputCurrent());

    pivotCurrentPositionLog.append(pivotAbs.getPosition());
    pivotTargetPositionLog.append(pivotTargetPostion);
    pivotAmpsLog.append(pivot.getOutputCurrent());

    rotateCurrentPositionLog.append(rotateAbs.getPosition());
    rotateTargetPositionLog.append(rotateTargetPostion);
    rotateAmpsLog.append(rotate.getOutputCurrent());

    scoreRotatePosPub.set(rotateAbs.getPosition());
    scoreRotateSetpointPub.set(rotateTargetPostion);
    scoreRotateAmpsPub.set(rotate.getOutputCurrent());

    scorePivotPosPub.set(pivotAbs.getPosition());
    scorePivotSetpointPub.set(pivotTargetPostion);
    scorePivotAmpsPub.set(pivot.getOutputCurrent());

    scoreClawPosPub.set(ClawAbs.getPosition());
    scoreClawSetpointPub.set(clawTargetPostion);
    scoreClawAmpsPub.set(Claw.getOutputCurrent());

    // //SmartDashboard.putNumber("ClawAbs", ClawAbs.getPosition());
    // SmartDashboard.putNumber("RotateAbs", rotateAbs.getPosition());
    // SmartDashboard.putNumber("PivotAbs", pivotAbs.getPosition());
    // //SmartDashboard.putNumber("ClawRencoder", Claw.getEncoder().getPosition());
    // SmartDashboard.putNumber("RotateRencoder", rotate.getEncoder().getPosition());
    // SmartDashboard.putNumber("PivotRencoder", pivot.getEncoder().getPosition());
    // SmartDashboard.putNumber("RotateTargetPosition", rotateTargetPostion);
    // SmartDashboard.putNumber("PivotTargetPosition", pivotTargetPostion);
    // SmartDashboard.putString("Score State", this.scoreState.toString());
  }
}
