// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class ControllerInputSubsystem extends SubsystemBase {
  /** Creates a new ControllerInputSubsystem. */

  CommandXboxController controller;
  SlewRateLimiter xLimiter = new SlewRateLimiter(5.0);
  SlewRateLimiter yLimiter = new SlewRateLimiter(5.0);

  public ControllerInputSubsystem(CommandXboxController controller) {
    this.controller = controller;
  }

  public double getRateLimitedXDrive(){
    return xLimiter.calculate(controller.getLeftX());
  }

  public double getRateLimitedYDrive(){
    return yLimiter.calculate(controller.getLeftY());
  }

  public double getAllianceDirectedXDrive(){
    if(DriverStation.getAlliance().get() == Alliance.Red){
      return -getRateLimitedXDrive();
    }
    return getRateLimitedXDrive();
  }

  public double getAllianceDirectedYDrive(){
    if(DriverStation.getAlliance().get() == Alliance.Red){
      return -getRateLimitedYDrive();
    }
    return getRateLimitedYDrive();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
