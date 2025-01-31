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
  private SparkMax elevator = new SparkMax(Constants.elevatorID, MotorType.kBrushless);
  private SparkClosedLoopController LoopyDoopy = elevator.getClosedLoopController();
  private RelativeEncoder eleEncoder = elevator.getEncoder();
  private SparkMax InLa = new SparkMax(Constants.InLaID, MotorType.kBrushless);
  private SparkClosedLoopController LoopyCallyGirls = InLa.getClosedLoopController();
  private RelativeEncoder PartyInTheUSA = InLa.getEncoder();
  private SparkMaxConfig AConfigSongWasOn = new SparkMaxConfig();
  private double targetPostionInLa = 0.0;
  private double targetPostion = 0.0;
  private SparkMaxConfig Configaroo = new SparkMaxConfig();
  private DigitalInput Inny = new DigitalInput(8);
    
  DataLog log = DataLogManager.getLog();
  DoubleLogEntry elePosLog = new DoubleLogEntry(log, "/elePos");
  /** Creates a new Elevator. */
  public Elevator() {
    eleEncoder.setPosition(0.0);
    Configaroo.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    Configaroo.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-1, 1)
   .p(0.0001, ClosedLoopSlot.kSlot1)
   .i(0, ClosedLoopSlot.kSlot1)
   .d(0, ClosedLoopSlot.kSlot1)
   .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
   .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    elevator.configure(Configaroo, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    PartyInTheUSA.setPosition(0.0);
    AConfigSongWasOn.encoder.positionConversionFactor(1).velocityConversionFactor(1);
    AConfigSongWasOn.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
   .p(1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-1, 1)
   .p(0.0001, ClosedLoopSlot.kSlot1)
   .i(0, ClosedLoopSlot.kSlot1)
   .d(0, ClosedLoopSlot.kSlot1)
   .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
   .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    InLa.configure(AConfigSongWasOn, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void setTargestPostion(double targetPostion){
    this.targetPostion = targetPostion;
  }

  public void InLA(double targetPostionInLa){
    this.targetPostionInLa = targetPostionInLa;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("EleEncoder", eleEncoder.getPosition());
    LoopyDoopy.setReference(targetPostion, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    SmartDashboard.putNumber("DroopyLoopy", targetPostion);
    SmartDashboard.putBoolean("Inny", Inny.get());

    elePosLog.append(eleEncoder.getPosition());

    LoopyCallyGirls.setReference(targetPostionInLa, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    SmartDashboard.putNumber("loopyCallyGirls", targetPostionInLa);

    if (!Inny.get()) {
      eleEncoder.setPosition(Constants.ElevatorHome);
    }
  }
}

