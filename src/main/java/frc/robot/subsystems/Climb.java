// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climb extends SubsystemBase {
  private SparkMax ClimbRotate = new SparkMax(Constants.ID_CLIMB_ROTATE, MotorType.kBrushless);
  private SparkMax ClimbWinch = new SparkMax(Constants.ID_CLIMB_WINCH, MotorType.kBrushless);
  private AbsoluteEncoder CRencoder = ClimbRotate.getAbsoluteEncoder();
  private AbsoluteEncoder CWencoder = ClimbWinch.getAbsoluteEncoder();
  private AbsoluteEncoderConfig CREconfig = new AbsoluteEncoderConfig();
  private AbsoluteEncoderConfig CWEconfig = new AbsoluteEncoderConfig();
  private SparkClosedLoopController CrotateLoopy = ClimbRotate.getClosedLoopController();
  private SparkClosedLoopController CwinchLoopy = ClimbWinch.getClosedLoopController();
  private SparkMaxConfig CRconfig = new SparkMaxConfig();
  private SparkMaxConfig CWConfig = new SparkMaxConfig();

  private double CRclimbingTargetPos = Constants.CLIMB_CLIMBING_POS, CRinTargetPos = Constants.CLIMB_IN_POS, CRoutTargetPos = Constants.CLIMB_OUT_POS;
    
  /** Creates a new Climb. */
  public Climb() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
