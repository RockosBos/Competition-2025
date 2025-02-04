// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  private SparkMax IntakeEle = new SparkMax(Constants.ID_INTAKE_ELEVATOR, MotorType.kBrushless);
  private SparkClosedLoopController IntakeLoopy = IntakeEle.getClosedLoopController();
  private RelativeEncoder IntakeEleEncoder = IntakeEle.getEncoder();
  private SparkMax ScoreEle = new SparkMax(Constants.ID_SCORE_ELEVATOR, MotorType.kBrushless);
  private SparkClosedLoopController ScoreEleLoopy = ScoreEle.getClosedLoopController();
  private RelativeEncoder ScoreEleEncoder = ScoreEle.getEncoder();
  private SparkMaxConfig ConfigScore = new SparkMaxConfig();
  private double targetPostionScoreInLa = 0.0;
  private double targetPostion = 0.0;
  private SparkMaxConfig Configaroo = new SparkMaxConfig();
  private DigitalInput InnyScory = new DigitalInput(8);
    
  DataLog log = DataLogManager.getLog();
  DoubleLogEntry elePosLog = new DoubleLogEntry(log, "/U/Elevator/EleIntakePos");
  /** Creates a new Elevator. */
  public Elevator() {
    IntakeEleEncoder.setPosition(0.0);
    Configaroo.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    Configaroo.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-0.1, 0.1);

    IntakeEle.configure(Configaroo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    ScoreEleEncoder.setPosition(0.0);
    ConfigScore.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    ConfigScore.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-0.1, 0.1);

    ScoreEle.configure(ConfigScore, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

    /**
     * Set Target Position for the Intake Elevator, measured in Rotations of relative encoder
     * 
     * @param targetPosition The target position for the intake elevator in Rotations
     */
  public void setIntakeTargetPostion(double targetPostion){
    this.targetPostion = targetPostion;
  }

    /**
     * Set Target Position for the Scoring Elevator, measured in Rotations of relative encoder
     * 
     * @param targetPosition The target position for the intake elevator in Rotations
     */
  public void setScoreTargetPosition(double targetPostionInLa){
    this.targetPostionScoreInLa = targetPostionInLa;
  }

  public boolean inPosition(){
    return false;
  }

  @Override
  public void periodic() {
    IntakeLoopy.setReference(targetPostion, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    ScoreEleLoopy.setReference(targetPostionScoreInLa, ControlType.kPosition, ClosedLoopSlot.kSlot1);

    //logging
    elePosLog.append(IntakeEleEncoder.getPosition());
  }
}