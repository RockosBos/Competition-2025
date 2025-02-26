// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.enums.CameraType;

public class CameraSubsystem extends SubsystemBase {

  CameraType cameraType;
  Pose3d pose3d = new Pose3d();
  String cameraName = "";
  PhotonCamera photonCamera;
  List<PhotonPipelineResult> resultList;
  PhotonTrackedTarget target;
  PhotonUtils utils;
  AprilTagFieldLayout aprilTagFieldLayout;
  Transform3d bestCameraToTarget;
  DoubleLogEntry logPoseX, logPoseY, logPoseT;
  double robotYaw;
  
  Pose2d currentPose = new Pose2d();
  PhotonPipelineResult result;
  Transform3d cameraToRobotPose = new Transform3d();

  StructPublisher<Pose2d> cameraPosePublisher;

  /** Creates a new CameraSubsystem. */
  public CameraSubsystem(CameraType cameraType, String cameraName, Transform3d cameraToRobotOffset, AprilTagFieldLayout fieldLayout) {
    this.cameraType = cameraType;
    this.cameraName = cameraName;
    this.aprilTagFieldLayout = fieldLayout;
    this.cameraToRobotPose = cameraToRobotOffset;
    DataLog log = DataLogManager.getLog();

    if(cameraType == CameraType.PHOTONVISION){
      photonCamera = new PhotonCamera(cameraName);
    }
    cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic(cameraName + "Pose Log", Pose2d.struct).publish();
  }

  public CameraSubsystem(CameraType cameraType, String cameraName) {
    this.cameraType = cameraType;
    this.cameraName = cameraName;
    DataLog log = DataLogManager.getLog();

    if(cameraType == CameraType.PHOTONVISION){
      aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
      photonCamera = new PhotonCamera(cameraName);
    }
    cameraPosePublisher = NetworkTableInstance.getDefault().getStructTopic(cameraName + "Pose Log", Pose2d.struct).publish();
  }

  public void setRobotYaw(double yaw){
    robotYaw = yaw;
  }

  public CameraType getCameraType(){
    return cameraType;
  }

  public Pose3d update3DPose(){
    if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
      pose3d = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), aprilTagFieldLayout.getTagPose(target.getFiducialId()).get(), cameraToRobotPose);
    }
    return this.pose3d;
  }

  public Pose2d update2DPose(){
    // if(CameraType.PHOTONVISION == cameraType){
    //   if(result.hasTargets()){
    //     target = result.getBestTarget();
    //   }
    // }
    
    currentPose = update3DPose().toPose2d();
    cameraPosePublisher.set(new Pose2d(new Translation2d(currentPose.getX(), currentPose.getY()), currentPose.getRotation()));
    return currentPose;
  }

  public Pose2d getPose2d(){
    return currentPose;
  }

  public boolean hasTarget(){

    return result.hasTargets();
  }

  @Override
  public void periodic() {
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        aprilTagFieldLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
      }
      else{
        aprilTagFieldLayout.setOrigin(OriginPosition.kRedAllianceWallRightSide);
      }
    }
    result = photonCamera.getLatestResult();

    if(CameraType.PHOTONVISION == cameraType){
      result = photonCamera.getLatestResult();
      if(result.hasTargets()){
        target = result.getBestTarget();
        currentPose = update2DPose();
      }
    }
    else{
      LimelightHelpers.SetRobotOrientation(cameraName, robotYaw, 0, 0, 0, 0, 0);
      currentPose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName).pose;
    }

  }
}
