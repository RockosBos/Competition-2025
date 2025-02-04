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

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Score extends SubsystemBase {

private SparkMax AgitateRheel = new SparkMax(Constants.ID_SCORE_AGITATE, MotorType.kBrushless);
private SparkMax Claw = new SparkMax(Constants.ID_SCORE_CLAW, MotorType.kBrushless);
private SparkMax UpDownPivot = new SparkMax(Constants.ID_SCORE_PIVOT, MotorType.kBrushless);
private SparkMax LiftyRighty = new SparkMax(Constants.ID_SCORE_ROTATE, MotorType.kBrushless);
private AbsoluteEncoder ClawAbs = Claw.getAbsoluteEncoder();
private AbsoluteEncoder UpDownPiviotAbs = UpDownPivot.getAbsoluteEncoder();
private AbsoluteEncoder LiftyRightyAbs = LiftyRighty.getAbsoluteEncoder();
private AbsoluteEncoderConfig ClawAbsConfig = new AbsoluteEncoderConfig();
private AbsoluteEncoderConfig UpDownPiviotAbsConfig = new AbsoluteEncoderConfig();
private AbsoluteEncoderConfig LiftyRightyAbsConfig = new AbsoluteEncoderConfig();
private SparkClosedLoopController ClawLoopy = Claw.getClosedLoopController();
private SparkClosedLoopController UpDownPiviotLoopy = UpDownPivot.getClosedLoopController();
private SparkClosedLoopController LiftyRightyLoopy = LiftyRighty.getClosedLoopController();
SparkMaxConfig ClawConfig = new SparkMaxConfig();
SparkMaxConfig UpDownPiviotConfig = new SparkMaxConfig();
SparkMaxConfig LiftyRighyConfig = new SparkMaxConfig();
private double targetPostionRotate = 0.0, targetPostionPivot = 0.0, targetPostionClaw = 0.0;


  /** Creates a new Score. */
  public Score() {
    ClawConfig.inverted(false);
    ClawAbsConfig.zeroOffset(0.728);
    ClawConfig.absoluteEncoder.apply(ClawAbsConfig);
    ClawConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-1, 1);
   
    Claw.configure(ClawConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    UpDownPiviotConfig.inverted(false);
    UpDownPiviotAbsConfig.zeroOffset(0.728);
    UpDownPiviotConfig.absoluteEncoder.apply(ClawAbsConfig);
    UpDownPiviotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-1, 1);

    UpDownPivot.configure(UpDownPiviotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //Change for test commit, remove if you see this
  }

  /**
   * Set Arm Rotate Target Position, There should be three states: Left, Right, and Center
   * 
   * @param rotateTargetPosition The Requested Set Position on the absolute encoder for the closed loop
   * controller to navigate to.
   */
  public boolean atRotateTarget(){
    if(Constants.THRESHOLD_SCORE_ROTATE_POS > targetPostionRotate){
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
    if (Constants.THRESHOLD_SCORE_PIVOT_POS > targetPostionPivot) {
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
    if (Constants.THRESHOLD_SCORE_CLAW_POS > targetPostionClaw) {
      return true;
    }
      return false;
  }

  public void setRotateTargetPostion(double targetPostionRotate){
    this.targetPostionRotate = targetPostionRotate;
  }

  public void setPivotTargetPostion(double targetPostionPivot){
    this.targetPostionPivot = targetPostionPivot;
  }

  public void setClawTargetPostion(double targetPostionClaw){
    this.targetPostionClaw = targetPostionClaw;
  }
    /**
   * Set Claw Target Position, There should be an Open and Closed Position.
   * 
   * @param agitatorRollerVoltage The Requested voltage to set Agitator voltage to.
   * 
   */
  public void setAgitatorRollerVoltage(double agitatorRollerVoltage){

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
    // This method will be called once per scheduler run
  }
}
