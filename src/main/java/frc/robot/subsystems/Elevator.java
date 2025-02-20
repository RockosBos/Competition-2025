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
  private DigitalInput InnyScory = new DigitalInput(8);
    
  DataLog log = DataLogManager.getLog();
  private DoubleLogEntry intakeElevatorAmpLog, intakeElevatorVoltageLog, intakeElevatorTargetPositionLog, intakeElevatorCurrentPositionLog;
  private DoubleLogEntry scoreElevatorAmpLog, scoreElevatorVoltageLog, scoreElevatorTargetPositionLog, scoreElevatorCurrentPositionLog;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private final NetworkTable table = inst.getTable("Elevator");
  private final DoublePublisher intakeElevatorPosPub = table.getDoubleTopic("elevator").publish(),
                                intakeElevatorSetpointPub = table.getDoubleTopic("elevator").publish(), 
                                scoreElevatorPosPub = table.getDoubleTopic("elevator").publish(),
                                scoreElevatorSetpointPub = table.getDoubleTopic("elevator").publish();

  /** Creates a new Elevator. */
  public Elevator() {
    IntakeEleEncoder.setPosition(0.0);
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

    ScoreEleEncoder.setPosition(0.0);
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

    intakeElevatorTargetPositionLog = new DoubleLogEntry(log, "/U/Elevator/intakeElevatorTargetPosition");
    intakeElevatorCurrentPositionLog = new DoubleLogEntry(log, "/U/Elevator/intakeElevatorCurrentPosition");

    scoreElevatorTargetPositionLog = new DoubleLogEntry(log, "/U/Elevator/scoreElevatorTargetPosition");
    scoreElevatorCurrentPositionLog = new DoubleLogEntry(log, "/U/Elevator/scoreElevatorCurrentPosition");
    
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
    ScoreEleLoopy.setReference(Constants.SCORE_ELEVATOR_L3_POS, ControlType.kPosition, ClosedLoopSlot.kSlot0);

    SmartDashboard.putNumber("EleTarget", targetPostion);
    SmartDashboard.putNumber("IntakeEleEnc", IntakeEle.getEncoder().getPosition());
    SmartDashboard.putNumber("ScoreEle", ScoreEleEncoder.getPosition());

    //logging
    intakeElevatorTargetPositionLog.append(IntakeEleEncoder.getPosition());;
    intakeElevatorCurrentPositionLog.append(targetPostion);

    intakeElevatorPosPub.set(IntakeEleEncoder.getPosition());
    intakeElevatorSetpointPub.set(targetPostion);

    scoreElevatorTargetPositionLog.append(ScoreEleEncoder.getPosition());;
    scoreElevatorCurrentPositionLog.append(targetPostionScoreInLa);

    scoreElevatorPosPub.set(ScoreEleEncoder.getPosition());
    scoreElevatorSetpointPub.set(targetPostionScoreInLa);
  }
}