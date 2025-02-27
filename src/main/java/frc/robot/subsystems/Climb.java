// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Climb extends SubsystemBase {
  private SparkMax WereClimbingToLA = new SparkMax(Constants.ID_CLIMB_ROTATE, MotorType.kBrushless);
  private AbsoluteEncoder ToLAencoder = WereClimbingToLA.getAbsoluteEncoder();
  private AbsoluteEncoderConfig ToLaeconfig = new AbsoluteEncoderConfig();
  private SparkClosedLoopController ToLALoopy = WereClimbingToLA.getClosedLoopController();
  private SparkMaxConfig ToLAconfig = new SparkMaxConfig();


  private double CRclimbingTargetPos = 0.0;
  /** Creates a new Climb. */
   public Climb() {
    ToLaeconfig.inverted(false);
    ToLaeconfig.zeroOffset(0.728);
    ToLAconfig.absoluteEncoder.apply(ToLaeconfig);
    ToLAconfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
     .p(0.1)
   .i(0)
   .d(0)
   .velocityFF(0)
  .outputRange(-1, 1);
   
   WereClimbingToLA.configure(ToLAconfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void ClimbInTargetPosition(double CRclimbingTargetPos){
    this.CRclimbingTargetPos = CRclimbingTargetPos;
  }

  public void ClimbClimbingTargetPosition(double CRclimbingTargetPos){
    this.CRclimbingTargetPos = CRclimbingTargetPos;
  }
   
  public void ClimbOutTargetPosition(double CRclimbingTargetPos){
    this.CRclimbingTargetPos = CRclimbingTargetPos;
  }

   @Override
   public void periodic() {

   }
  }