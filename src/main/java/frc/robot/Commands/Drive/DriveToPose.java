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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToPose extends Command {
  CommandSwerveDrivetrain driveSubsystem;

  PIDController xController, yController, tController;
  double xDrive, yDrive, tDrive;

  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  /** Creates a new PathplannerToPose. */
  public DriveToPose(CommandSwerveDrivetrain driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    addRequirements(driveSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    xController = new PIDController(0.1, 0, 0);
    yController = new PIDController(0.1, 0, 0);
    tController = new PIDController(0.1, 0, 0);
    xDrive = 0;
    yDrive = 0;
    tDrive = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    xDrive = xController.calculate(driveSubsystem.samplePoseAt(Utils.getCurrentTimeSeconds()).get().getX(), 0);
    yDrive = yController.calculate(driveSubsystem.samplePoseAt(Utils.getCurrentTimeSeconds()).get().getY(), 0);
    tDrive = tController.calculate(driveSubsystem.getPigeon2().getYaw().getValueAsDouble(), 0);

    // SmartDashboard.putNumber("xDrive", xDrive);
    // SmartDashboard.putNumber("yDrive", yDrive);
    // SmartDashboard.putNumber("tDrive", tDrive);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    xDrive = 0.0;
    yDrive = 0.0;
    tDrive = 0.0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
