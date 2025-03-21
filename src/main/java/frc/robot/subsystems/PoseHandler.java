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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LOADING_POSES;
import frc.robot.Constants.SCORING_POSES;

public class PoseHandler extends SubsystemBase {

  PIDController xController, yController, tController, xLoadingController, yLoadingController, tLoadingController;
  double xDrive, yDrive, tDrive, angle, loadingxDrive, loadingyDrive, loadingtDrive, loadingAngle;
  double  nearCenterDist = 0.0, nearLeftDist = 0.0, nearRightDist = 0.0,
          farCenterDist = 0.0, farLeftDist = 0.0, farRightDist = 0.0,
          loadingLeftDist = 0.0, loadingRightDist = 0.0;
  double closestScoringPoseX = 7.0, closestScoringPoseY = 4.0, closestScoringPoseT = 0.0, closestScoringDist = 0.0;
  double closestloadingPoseX = 1.654, closestLoadingPoseY = 7.436, closestLoadingPoseT = 120.0, closestLoadingDist = 0.0;
  String closestScoringLocation = "FC", closestLoadinglocation = "L";
  AprilTagFieldLayout aprilTagFieldLayout;
  StructPublisher<Pose2d> closestPosePublisher, closestLoadingPosePublisher;
  Timer loopTimer = new Timer();
  
  
  private final double maxSpeedX = 0.15, maxSpeedY = 0.15, maxSpeedT = 0.5;
  /** Creates a new PoseHandler. */
  public PoseHandler() {
    xController = new PIDController(3.5, 0, 0);
    yController = new PIDController(3.5, 0, 0);
    tController = new PIDController(0.1, 0, 0);
    xDrive = 0;
    yDrive = 0;
    tDrive = 0;
    aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    closestPosePublisher = NetworkTableInstance.getDefault().getStructTopic("PoseHandlerScoring", Pose2d.struct).publish();
    closestLoadingPosePublisher = NetworkTableInstance.getDefault().getStructTopic("PoseHandlerLoading", Pose2d.struct).publish();
  }

  public boolean xInPosition(){
    if(Math.abs(xDrive) < 0.01){
      return true;
    }
    return false;
  }

  public boolean yInPosition(){
    if(Math.abs(yDrive) < 0.01){
      return true;
    }
    return false;
  }

  public boolean tInPosition(){
    if(Math.abs(tDrive) < 0.01){
      return true;
    }
    return false;
  }

  public boolean inPosition(){
    return xInPosition() && yInPosition() && tInPosition();
  }

  public double getXController(Pose2d pose){
      xDrive = xController.calculate(pose.getX(), closestScoringPoseX);
      if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == Alliance.Red){
          xDrive = -xDrive;
        }
      }
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
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        yDrive = -yDrive;
      }
    }
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
    if(angle < -160){
      angle = angle + 360;
    }
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

  public double getXControllerLoading(Pose2d pose){
      xDrive = xController.calculate(pose.getX(), closestScoringPoseX);
      if(DriverStation.getAlliance().isPresent()){
        if(DriverStation.getAlliance().get() == Alliance.Red){
          xDrive = -xDrive;
        }
      }
      if(xDrive > maxSpeedX){
        return maxSpeedX;
      }
      if(xDrive < -maxSpeedX){
        return -maxSpeedX;
      }
      
      return xDrive;
  }

  public double getYControllerLoading(Pose2d pose){
    yDrive = yController.calculate(pose.getY(), closestScoringPoseY);
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        yDrive = -yDrive;
      }
    }
    if(yDrive > maxSpeedY){
      return maxSpeedY;
    }
    if(yDrive < -maxSpeedY){
      return -maxSpeedY;
    }

    return yDrive;
  }

public double getTControllerLoading(double angle){
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

  public void updateNearestScorePose(Pose2d pose){
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

    // if(DriverStation.getAlliance().isPresent()){
    //   if(DriverStation.getAlliance().get() == Alliance.Blue){
    //     closestScoringPoseT = closestScoringPoseT - 180.0;
    //   }
    // }

    //closestScoringPoseT = closestScoringPoseT - 180.0;
  }

  // public void updateNearestLoadingPose(Pose2d pose){
  //   loadingLeftDist = getDistanceToCoord(pose.getX(), pose.getY(), LOADING_POSES.LEFT.X, LOADING_POSES.LEFT.Y);
  //   loadingRightDist = getDistanceToCoord(pose.getX(), pose.getY(), LOADING_POSES.RIGHT.X, LOADING_POSES.RIGHT.Y);

  //   if(loadingLeftDist < loadingRightDist){
  //     closestLoadingDist = loadingLeftDist;
  //     closestLoadinglocation = "L";
  //     closestScoringPoseX = LOADING_POSES.LEFT.X;
  //     closestScoringPoseY = LOADING_POSES.LEFT.Y;
  //     closestScoringPoseT = LOADING_POSES.LEFT.T;
  //   }
  //   else{
  //     closestLoadingDist = loadingRightDist;
  //     closestLoadinglocation = "R";
  //     closestScoringPoseX = LOADING_POSES.RIGHT.X;
  //     closestScoringPoseY = LOADING_POSES.RIGHT.Y;
  //     closestScoringPoseT = LOADING_POSES.RIGHT.T;
  //   }
  // }
  
  public AprilTagFieldLayout getAprilTagFieldLayout(){
    return aprilTagFieldLayout;
  }

  @Override
  public void periodic() {

    // if(loopTimer.get() > 1.0){
    //   closestPosePublisher.set(new Pose2d(new Translation2d(closestScoringPoseX, closestScoringPoseY), new Rotation2d(closestScoringPoseT)));
    //   //closestLoadingPosePublisher.set(new Pose2d(new Translation2d(closestloadingPoseX, closestLoadingPoseY), new Rotation2d(closestLoadingPoseT)));
    //   loopTimer.reset();
    // }
    
    closestPosePublisher.set(new Pose2d(new Translation2d(closestScoringPoseX, closestScoringPoseY), new Rotation2d(closestScoringPoseT)));
    
    // SmartDashboard.putNumber("t Drive", tDrive);
    // SmartDashboard.putNumber("angle", angle);
    // SmartDashboard.putNumber("closestScroingPoseT", closestScoringPoseT);
    // SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());

  }
}
