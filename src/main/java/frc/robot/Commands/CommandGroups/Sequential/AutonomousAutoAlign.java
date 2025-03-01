// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Drive.DriveToNearestScore;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.PoseHandler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutonomousAutoAlign extends SequentialCommandGroup {
  CommandSwerveDrivetrain drivetrain;
  PoseHandler PoseHandlerSubsystem;

      //Setup for telemetry and Swerve Drive

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  /** Creates a new AutonomousAutoAlign. */
  public AutonomousAutoAlign(CommandSwerveDrivetrain driveSubsystem, PoseHandler PoseHandlerSubsystem) {
    

    this.drivetrain = driveSubsystem;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelDeadlineGroup(
        new DriveToNearestScore(drivetrain, PoseHandlerSubsystem), 
        drivetrain.applyRequest(() ->
            drive.withVelocityX(PoseHandlerSubsystem.getXController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(PoseHandlerSubsystem.getYController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(PoseHandlerSubsystem.getTController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get().getRotation().getDegrees()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        )
      ),
      drivetrain.applyRequest(() ->
      drive.withVelocityX(0.0 * MaxSpeed) // Drive forward with negative Y (forward)
          .withVelocityY(0.0 * MaxSpeed) // Drive left with negative X (left)
          .withRotationalRate(0.0 * MaxAngularRate) // Drive counterclockwise with negative X (left)
      )
    );
  }
}
