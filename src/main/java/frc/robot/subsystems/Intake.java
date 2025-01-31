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
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private SparkMax IntakeIn = new SparkMax(Constants.IntakeID, MotorType.kBrushless);
  private SparkMax IntakeRotate = new SparkMax(Constants.IntakeRotateID, MotorType.kBrushless);
  private SparkClosedLoopController BetterLoppyDoopy = IntakeRotate.getClosedLoopController();
  private AbsoluteEncoder IAE = IntakeRotate.getAbsoluteEncoder();
  private AbsoluteEncoderConfig IAESLKG = new AbsoluteEncoderConfig();
  private double voltage = 0.0;
  SparkMaxConfig config = new SparkMaxConfig();
  SparkMaxConfig AC = new SparkMaxConfig();
  private double maxCurrent = 0.0;
  private double targetPostion = 0.0;
  /** Creates a new Intake. */
  public Intake() {
    config.inverted(false);
    IAESLKG.zeroOffset(0.728);
    AC.absoluteEncoder.apply(IAESLKG);
    AC.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
    .p(0.1)
   .i(0)
   .d(0)
   .velocityFF(0)
   .outputRange(-1, 1)
   .p(0.0001, ClosedLoopSlot.kSlot1)
   .i(0, ClosedLoopSlot.kSlot1)
   .d(0, ClosedLoopSlot.kSlot1)
   .velocityFF(1.0 / 5767, ClosedLoopSlot.kSlot1)
   .outputRange(-1, 1, ClosedLoopSlot.kSlot1);
    IntakeRotate.configure(AC, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    IntakeIn.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setMoterVoltage(double voltage){
    this.voltage = voltage;
  }

  public void setTargestPostion(double targetPostion){
    this.targetPostion = targetPostion;
  }

  @Override
  public void periodic() {
    if (IntakeIn.getOutputCurrent() > 10) {
      voltage = 0.25;
    }
    IntakeIn.set(voltage);
    SmartDashboard.putNumber("Intakecurrent", IntakeIn.getOutputCurrent());
    SmartDashboard.putNumber("IAE", IAE.getPosition());
    SmartDashboard.putNumber("Target Position", targetPostion);
     BetterLoppyDoopy.setReference(targetPostion, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
