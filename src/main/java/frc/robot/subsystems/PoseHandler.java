// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseHandler extends SubsystemBase {

  PIDController xController, yController, tController;
  double xDrive, yDrive, tDrive;
  private final double maxSpeedX = 0.2, maxSpeedY = 0.2, maxSpeedT = 0.2;
  /** Creates a new PoseHandler. */
  public PoseHandler() {
    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
    tController = new PIDController(0.1, 0, 0);
    xDrive = 0;
    yDrive = 0;
    tDrive = 0;
  }

  public double getXController(Pose2d pose){
      xDrive = xController.calculate(pose.getX(), 0);
      if(xDrive > maxSpeedX){
        return maxSpeedX;
      }
      return xDrive;
  }

  public double getYController(Pose2d pose){
    yDrive = yController.calculate(pose.getY(), 0);
    if(yDrive > maxSpeedY){
      return maxSpeedY;
    }
    return yDrive;
  }

  public double getTController(Pose2d pose){
    tDrive = yController.calculate(pose.getRotation().getDegrees(), 0);
    if(tDrive > maxSpeedT){
      return maxSpeedT;
    }
    return tDrive;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
