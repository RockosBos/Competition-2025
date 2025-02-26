// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.Utils;
import com.google.flatbuffers.Constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SCORING_POSES;

public class PoseHandler extends SubsystemBase {

  PIDController xController, yController, tController;
  double xDrive, yDrive, tDrive, angle;
  double  nearCenterDist = 0.0, nearLeftDist = 0.0, nearRightDist = 0.0,
          farCenterDist = 0.0, farLeftDist = 0.0, farRightDist = 0.0;
  double closestScoringPoseX = 7.0, closestScoringPoseY = 4.0, closestScoringPoseT = 0.0, closestScoringDist = 0.0;
  String closestScoringLocation = "FC";
  AprilTagFieldLayout aprilTagFieldLayout;
  
  
  private final double maxSpeedX = 0.2, maxSpeedY = 0.2, maxSpeedT = 0.75;
  /** Creates a new PoseHandler. */
  public PoseHandler() {
    xController = new PIDController(1.2, 0, 0);
    yController = new PIDController(1.2, 0, 0);
    tController = new PIDController(0.07, 0, 0);
    xDrive = 0;
    yDrive = 0;
    tDrive = 0;
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

  }

  public double getXController(Pose2d pose){
      xDrive = xController.calculate(pose.getX(), closestScoringPoseX);
      if(xDrive > maxSpeedX){
        return maxSpeedX;
      }
      if(xDrive < -maxSpeedX){
        return -maxSpeedX;
      }
      return xDrive;
  }

  public double getYController(Pose2d pose){
    yDrive = yController.calculate(pose.getY(), closestScoringPoseY);
    if(yDrive > maxSpeedY){
      return maxSpeedY;
    }
    if(yDrive < -maxSpeedY){
      return -maxSpeedY;
    }
    return yDrive;
  }

  public double getTController(double angle){
    this.angle = angle;
    tDrive = tController.calculate(angle, closestScoringPoseT);
    //return 0;
    if(tDrive > maxSpeedT){
      return maxSpeedT;
    }
    if(tDrive < -maxSpeedT){
      return -maxSpeedT;
    }
    return tDrive;
  }

  public double getDistanceToCoord(double currentX, double currentY, double targetX, double targetY){
    return Math.sqrt(Math.pow((targetX - currentX), 2) + Math.pow((targetY - currentY), 2));
  }

  public void updateNearestPose(Pose2d pose){
    nearCenterDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.CENTER_NEAR.X, SCORING_POSES.CENTER_NEAR.Y);
    nearLeftDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.LEFT_NEAR.X, SCORING_POSES.LEFT_NEAR.Y);
    nearRightDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.RIGHT_NEAR.X, SCORING_POSES.RIGHT_NEAR.Y);
    farRightDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.RIGHT_FAR.X, SCORING_POSES.RIGHT_FAR.Y);
    farCenterDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.CENTER_FAR.X, SCORING_POSES.CENTER_FAR.Y);
    farLeftDist = getDistanceToCoord(pose.getX(), pose.getY(), SCORING_POSES.LEFT_FAR.X, SCORING_POSES.LEFT_FAR.Y);
  
    if(nearCenterDist < nearLeftDist){
      closestScoringDist = nearCenterDist;
      closestScoringLocation = "NC";
    }
    else{
      closestScoringDist = nearLeftDist;
      closestScoringLocation = "NL";
    }
    if(nearRightDist < closestScoringDist){
      closestScoringDist = nearRightDist;
      closestScoringLocation = "NR";
    }
    if(farLeftDist < closestScoringDist){
      closestScoringDist = farLeftDist;
      closestScoringLocation = "FL";
    }
    if(farCenterDist < closestScoringDist){
      closestScoringDist = farCenterDist;
      closestScoringLocation = "FC";
    }
    if(farRightDist < closestScoringDist){
      closestScoringDist = farRightDist;
      closestScoringLocation = "FR";
    }
    
    switch (closestScoringLocation) {
      case "NC":
        closestScoringPoseX = SCORING_POSES.CENTER_NEAR.X;
        closestScoringPoseY = SCORING_POSES.CENTER_NEAR.Y;
        closestScoringPoseT = SCORING_POSES.CENTER_NEAR.T;
        break;

      case "NL":
        closestScoringPoseX = SCORING_POSES.LEFT_NEAR.X;
        closestScoringPoseY = SCORING_POSES.LEFT_NEAR.Y;
        closestScoringPoseT = SCORING_POSES.LEFT_NEAR.T;
        break;

      case "NR":
        closestScoringPoseX = SCORING_POSES.RIGHT_NEAR.X;
        closestScoringPoseY = SCORING_POSES.RIGHT_NEAR.Y;
        closestScoringPoseT = SCORING_POSES.RIGHT_NEAR.T;
        break;

      case "FC":
        closestScoringPoseX = SCORING_POSES.CENTER_FAR.X;
        closestScoringPoseY = SCORING_POSES.CENTER_FAR.Y;
        closestScoringPoseT = SCORING_POSES.CENTER_FAR.T;
        break;

      case "FL":
        closestScoringPoseX = SCORING_POSES.LEFT_FAR.X;
        closestScoringPoseY = SCORING_POSES.LEFT_FAR.Y;
        closestScoringPoseT = SCORING_POSES.LEFT_FAR.T;
        break;

      case "FR":
        closestScoringPoseX = SCORING_POSES.RIGHT_FAR.X;
        closestScoringPoseY = SCORING_POSES.RIGHT_FAR.Y;
        closestScoringPoseT = SCORING_POSES.RIGHT_FAR.T;
        break;
    
      default:
        break;
    }

  }

  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return aprilTagFieldLayout;
  }

  @Override
  public void periodic() {
    
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber("xController", xDrive);
    // SmartDashboard.putNumber("yController", yDrive);
    // SmartDashboard.putNumber("tController", tDrive);
    // SmartDashboard.putNumber("PigeonAngle", angle);
    // SmartDashboard.putNumber("nearCenterDistance", nearCenterDist);
    // SmartDashboard.putNumber("nearLeftDistance", nearLeftDist);
    // SmartDashboard.putNumber("nearRightDistance", nearRightDist);
    // SmartDashboard.putNumber("farCenterDistance", farCenterDist);
    // SmartDashboard.putNumber("farLeftDistance", farLeftDist);
    // SmartDashboard.putNumber("farRightDistance", farRightDist);
    SmartDashboard.putNumber("closestScoringPoseX", closestScoringPoseX);
    SmartDashboard.putNumber("closestScoringPoseY", closestScoringPoseY);
    SmartDashboard.putString("Closest Pose", closestScoringLocation);

  }
}
