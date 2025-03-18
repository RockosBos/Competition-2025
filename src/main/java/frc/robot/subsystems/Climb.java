// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.config.ServoChannelConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climb extends SubsystemBase {
  private SparkMax WereClimbingToLA = new SparkMax(Constants.ID_CLIMB_ROTATE, MotorType.kBrushless);
  private RelativeEncoder toLAencoder = WereClimbingToLA.getEncoder();
  private SparkClosedLoopController ToLALoopy = WereClimbingToLA.getClosedLoopController();
  private SparkMaxConfig ToLAconfig = new SparkMaxConfig();
  private double errWereNotInLA = 0.0;
  private boolean wereReadyToClimbToLA = false;
  private Servo climbingToLAservo = new Servo(Constants.SERVO_ID);
  private double setServoPosClimbing = Constants.SERVO_UNLOCKED;
  private double CRclimbingTargetPos = 0.0;

  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private final NetworkTable table = inst.getTable("Climb");
  private final DoublePublisher climbPosPub = table.getDoubleTopic("climbPos").publish(),
                                climbSetpointPub = table.getDoubleTopic("climbSetpointPub").publish(),
                                climbAmpsPub = table.getDoubleTopic("ClimbAps").publish(),
                                climbServoPub = table.getDoubleTopic("ClimbServo").publish();
  /** Creates a new Climb. */
   public Climb() {
    ToLAconfig.inverted(Constants.INVERT_CLIMB_ROTATE);
    ToLAconfig.idleMode(Constants.IDELMODE_CLIMB);
    ToLAconfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
     .p(Constants.P_CLIMB)
    .i(0)
   .d(0)
   .velocityFF(0)
  .outputRange(Constants.MIN_OUTPUT_CLIMB, Constants.MAX_OUTPUT_CLIMB);
   
   WereClimbingToLA.configure(ToLAconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setClimbTargetPosition(double CRclimbingTargetPos){
    this.CRclimbingTargetPos = CRclimbingTargetPos;
  }

  public boolean areWeInLA(){
    if (errWereNotInLA < Constants.THRESHOLD_CLIMB_POS) {
      return true;
    } 
      return false;
  }

  public void areWeReadyToClimbToLA(boolean wereReadyToClimbToLA){
    this.wereReadyToClimbToLA = wereReadyToClimbToLA;
  }

  public void setServoClimbingToLAPos(double setServoPosClimbing){
    this.setServoPosClimbing = setServoPosClimbing;
  }

   @Override
   public void periodic() {
    errWereNotInLA = Math.abs(CRclimbingTargetPos - toLAencoder.getPosition());
    ToLALoopy.setReference(CRclimbingTargetPos, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    climbingToLAservo.setAngle(setServoPosClimbing);

    climbPosPub.set(toLAencoder.getPosition());
    climbSetpointPub.set(CRclimbingTargetPos);
    climbAmpsPub.set(WereClimbingToLA.getOutputCurrent());
    climbServoPub.set(setServoPosClimbing);
   }
  }