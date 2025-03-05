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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.ControlState;

public class Elevator extends SubsystemBase {
  private SparkMax IntakeEle = new SparkMax(Constants.ID_INTAKE_ELEVATOR, MotorType.kBrushless);
  private SparkClosedLoopController IntakeLoopy = IntakeEle.getClosedLoopController();
  private RelativeEncoder IntakeEleEncoder = IntakeEle.getEncoder();
  private SparkMax ScoreEle = new SparkMax(Constants.ID_SCORE_ELEVATOR, MotorType.kBrushless);
  private SparkClosedLoopController ScoreEleLoopy = ScoreEle.getClosedLoopController();
  private RelativeEncoder ScoreEleEncoder = ScoreEle.getEncoder();
  private SparkMaxConfig ConfigScore = new SparkMaxConfig();
  private double targetPostionScoreInLa = Constants.SCORE_ELEVATOR_INTAKE_POSITION;
  private double targetPostion = Constants.INTAKE_ELEVATOR_FLOOR_INTAKE_POS;
  private SparkMaxConfig Configaroo = new SparkMaxConfig();
  private DigitalInput InEleDiInput = new DigitalInput(4);
  private DigitalInput ScoreEleDiInput = new DigitalInput(7);

  private DigitalInput scoreElevatorProximity = new DigitalInput(Constants.SCORE_ELEVATOR_PROXIMITY_SENSOR);
  private DigitalInput intakeElevatorProximity = new DigitalInput(Constants.INTAKE_ELEVATOR_PROXIMITY_SENSOR);

  private ControlState intakeEleControlState = ControlState.CLOSEDLOOP;
  private ControlState scoreEleControlState = ControlState.CLOSEDLOOP;

  private double intakeEleVoltage = 0.0, scoreEleVoltage = 0.0;
  private InterpolatingDoubleTreeMap driveSpeedLimiter = new InterpolatingDoubleTreeMap();
  private double driveSpeedLimit = 1.0;
    
  DataLog log = DataLogManager.getLog();
  private DoubleLogEntry intakeElevatorAmpLog, intakeElevatorVoltageLog, intakeElevatorTargetPositionLog, intakeElevatorCurrentPositionLog;
  private DoubleLogEntry scoreElevatorAmpLog, scoreElevatorVoltageLog, scoreElevatorTargetPositionLog, scoreElevatorCurrentPositionLog;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final NetworkTable table = inst.getTable("Elevator");
  private final DoublePublisher intakeElevatorPosPub = table.getDoubleTopic("intakeElevatorPos").publish(),
                                intakeElevatorSetpointPub = table.getDoubleTopic("intakeElevatorSetpoint").publish(),
                                intakeElevatorAmpsPub = table.getDoubleTopic("intakeElevatorAmps").publish(), 
                                scoreElevatorPosPub = table.getDoubleTopic("scoreElevatorPos").publish(),
                                scoreElevatorSetpointPub = table.getDoubleTopic("scoreElevatorSetpoint").publish(),
                                scoreElevatorAmpsPub = table.getDoubleTopic("scoreElevatorAmps").publish();
  private final BooleanPublisher scoreElevatorSensorPub = table.getBooleanTopic("scoreElevatorSensor").publish(),
                                 intakeElevatorSensorPub = table.getBooleanTopic("intakeElevatorSensor").publish();


  /** Creates a new Elevator. */
  public Elevator() {
    //IntakeEleEncoder.setPosition(0.0);
    Configaroo.idleMode(Constants.IDLEMODE_INTAKE_ELEVATOR);
    Configaroo.closedLoopRampRate(Constants.RAMPRATE_INTAKE_ELEVATOR);
    Configaroo.smartCurrentLimit(Constants.CURRENTLIMIT_INTAKE_ELEVATOR);
    Configaroo.inverted(Constants.INVERT_INTAKE_ELEVATOR);
    Configaroo.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(Constants.P_INTAKE_ELEVATOR)
   .i(0)
   .d(0)
   .velocityFF(0.0)
   .outputRange(Constants.MIN_OUTPUT_INTAKE_ELEVATOR, Constants.MAX_OUTPUT_INTAKE_ELEVATOR);

    IntakeEle.configure(Configaroo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //ScoreEleEncoder.setPosition(0.0);
    ConfigScore.idleMode(Constants.IDLEMODE_SCORE_ELEVATOR);
    ConfigScore.closedLoopRampRate(Constants.RAMPRATE_SCORE_ELEVATOR);
    ConfigScore.smartCurrentLimit(Constants.CURRENTLIMIT_SCORE_ELEVATOR);
    ConfigScore.inverted(Constants.INVERT_SCORE_ELEVATOR);
    ConfigScore.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(Constants.P_SCORE_ELEVATOR)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(Constants.MIN_OUTPUT_SCORE_ELEVATOR, Constants.MAX_OUTPUT_SCORE_ELEVATOR);

    ScoreEle.configure(ConfigScore, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //Drive Speed Limiter Mapping
    driveSpeedLimiter.put(0.0, 0.9);
    driveSpeedLimiter.put(30.0, 0.8);
    driveSpeedLimiter.put(60.0, 0.65);
    driveSpeedLimiter.put(100.0, 0.55);
    driveSpeedLimiter.put(130.0, 0.4);
    driveSpeedLimiter.put(160.0, 0.25);

    intakeElevatorTargetPositionLog = new DoubleLogEntry(log, "/U/Elevator/intakeElevatorTargetPosition");
    intakeElevatorCurrentPositionLog = new DoubleLogEntry(log, "/U/Elevator/intakeElevatorCurrentPosition");
    intakeElevatorAmpLog = new DoubleLogEntry(log, "/U/Elevator/intakeElevatorAmps");

    scoreElevatorTargetPositionLog = new DoubleLogEntry(log, "/U/Elevator/scoreElevatorTargetPosition");
    scoreElevatorCurrentPositionLog = new DoubleLogEntry(log, "/U/Elevator/scoreElevatorCurrentPosition");
    scoreElevatorAmpLog = new DoubleLogEntry(log, "/U/Elevator/scoreElevatorAmps");
    
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
    return (intakeEleInPosition() && scoreEleInPosition());
  }

  public boolean intakeEleInPosition(){
    if(Math.abs(IntakeEleEncoder.getPosition() - targetPostion) < Constants.THRESHOLD_ELEVATOR_INTAKE_POS){
      return true;
    }
    return false;
  }

  public boolean scoreEleInPosition(){
    if(Math.abs(ScoreEleEncoder.getPosition() - targetPostionScoreInLa) < Constants.THRESHOLD_ELEVATOR_SCORE_POS){
      return true;
    }
    return false;
  }

  public void setScoreEleSpeedLimits(double minSpeed, double maxSpeed){
    ConfigScore.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(minSpeed, maxSpeed);
    ScoreEle.configure(ConfigScore, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public void setIntakeEleSpeedLimits(double minSpeed, double maxSpeed){
    Configaroo.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(minSpeed, maxSpeed);
    IntakeEle.configure(Configaroo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

  }

  public double getIntakeEleCurrent(){
    return IntakeEle.getOutputCurrent();
  }

  public double getScoreEleCurrent(){
    return ScoreEle.getOutputCurrent();
  }

  public void setIntakeEleControlState(ControlState controlState){
    intakeEleControlState = controlState;
  }

  public void setScoreEleControlState(ControlState controlState){
    scoreEleControlState = controlState;
  }

  public void setIntakeEleVoltage(double voltage){
    intakeEleVoltage = voltage;
  }

  public void setScoreEleVoltage(double voltage){
    scoreEleVoltage = voltage;
  }

  public void resetIntakeEle(){
    IntakeEleEncoder.setPosition(0.0);
  }

  public void resetScoreEle(){
    ScoreEleEncoder.setPosition(0.0);
  }

  public double getScoreEleRelativePosition(){
    return ScoreEleEncoder.getPosition();
  }

  public double getDriveSpeedLimit(){
    return driveSpeedLimit;
  }

  public boolean scoreElevatorSensor(){
    return !scoreElevatorProximity.get();
  }

  public boolean intakeElevatorSensor(){
    return !intakeElevatorProximity.get();
  }

  @Override
  public void periodic() {
    if(intakeEleControlState == ControlState.CLOSEDLOOP){
      IntakeLoopy.setReference(targetPostion, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    else{
      IntakeEle.setVoltage(intakeEleVoltage);
    }

    if(scoreEleControlState == ControlState.CLOSEDLOOP){
      ScoreEleLoopy.setReference(targetPostionScoreInLa, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }
    else{
      ScoreEle.setVoltage(scoreEleVoltage);
    }

    //Drive Speed Limiter
    driveSpeedLimit = driveSpeedLimiter.get(ScoreEleEncoder.getPosition());

    // SmartDashboard.putNumber("EleTarget", targetPostion);
    // SmartDashboard.putNumber("IntakeEleEnc", ScoreEle.getEncoder().getPosition());
    // SmartDashboard.putNumber("EleScoreTarget", targetPostionScoreInLa);
    // SmartDashboard.putNumber("ScoreEleEnc", ScoreEle.getEncoder().getPosition());

    //logging
    intakeElevatorTargetPositionLog.append(IntakeEleEncoder.getPosition());;
    intakeElevatorCurrentPositionLog.append(targetPostion);
    intakeElevatorAmpLog.append(IntakeEle.getOutputCurrent());

    intakeElevatorPosPub.set(IntakeEleEncoder.getPosition());
    intakeElevatorSetpointPub.set(targetPostion);

    scoreElevatorTargetPositionLog.append(ScoreEleEncoder.getPosition());;
    scoreElevatorCurrentPositionLog.append(targetPostionScoreInLa);
    scoreElevatorAmpLog.append(ScoreEle.getOutputCurrent());

    scoreElevatorPosPub.set(ScoreEleEncoder.getPosition());
    scoreElevatorSetpointPub.set(targetPostionScoreInLa);

    scoreElevatorSensorPub.set(scoreElevatorSensor());
    intakeElevatorSensorPub.set(intakeElevatorSensor());
  }
}