// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseHandler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToNearestScore extends Command {
  CommandSwerveDrivetrain driveSubsystem;
  PoseHandler poseHandler;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final Timer loopTimer = new Timer();
  
  /** Creates a new PathplannerToPose. */
  public DriveToNearestScore(CommandSwerveDrivetrain driveSubsystem, PoseHandler poseHandler) {
    this.driveSubsystem = driveSubsystem;
    this.poseHandler = poseHandler;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    loopTimer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(loopTimer.get() > 0.5){
      //poseHandler.updateNearestScorePose(driveSubsystem.samplePoseAt(Utils.getCurrentTimeSeconds()).get());
      loopTimer.reset();
    }
    poseHandler.updateNearestScorePose(driveSubsystem.samplePoseAt(Utils.getCurrentTimeSeconds()).get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    loopTimer.stop();
    System.out.println("Command Drive to Score Completed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return poseHandler.inPosition();
  }
}
