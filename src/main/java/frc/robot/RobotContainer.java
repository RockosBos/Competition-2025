// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CommandGroups.Sequential.L1;
import frc.robot.Commands.CommandGroups.Sequential.L2;
import frc.robot.Commands.CommandGroups.Sequential.L3;
import frc.robot.Commands.CommandGroups.Sequential.L4;
import frc.robot.Commands.Elevator.EleHandoffPos;
import frc.robot.Commands.Elevator.EleIntakePos;
import frc.robot.Commands.Elevator.EleL1Position;
import frc.robot.Commands.Elevator.EleL3Position;
import frc.robot.Commands.Intake.FloorIntakePosition;
import frc.robot.Commands.Intake.HandOffIntakePos;
import frc.robot.Commands.Intake.LoadingIntakePosition;
import frc.robot.Commands.Intake.OutfeedRoller;
import frc.robot.Commands.Score.AgitatorOn;
import frc.robot.Commands.Score.ClawClosed;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.HandoffScorePosition;
import frc.robot.Commands.Score.ScoreLeftState;
import frc.robot.Commands.Score.ScoreRightState;
import frc.robot.Commands.Score.ScoreSetCenter;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

public class RobotContainer {

    //Setup for telemetry and Swerve Drive

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    //Define Controllers

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final CommandXboxController operaterController = new CommandXboxController(1);

    //Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final Intake intakeSubsystem = new Intake();
    private final Elevator elevatorSubsytem = new Elevator();
    private final Score scoreSubsystem = new Score();

    //Triggers
    private final Trigger driverLeftTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);
    private final Trigger driverRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);
    private final Trigger operatorLeftTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);
    private final Trigger operatorRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);

    private final Trigger hasCoral = new Trigger(() -> intakeSubsystem.hasCoral());

    //Chooser for Autonomous Modes
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        //Data Log Initialization
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        //Named Commands for Pathplanner
        //NamedCommands.registerCommand("FloorIntakePosition", new FloorIntakePosition(intakeSubsystem, elevatorSubsytem, scoreSubsystem));

        //Auto Mode Setup
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        configureBindings();
    }

    private void configureBindings() {

        //Driver Controller

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        ));

        driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        //Operator Controller

        operaterController.x().onTrue(new HandOffIntakePos(intakeSubsystem));
        operaterController.a().onTrue(new HandoffScorePosition(scoreSubsystem));
        operaterController.b().onTrue(new EleHandoffPos(elevatorSubsytem));
        operaterController.y().onTrue(new ClawClosed(scoreSubsystem));
        operaterController.rightBumper().onTrue(new EleL3Position(elevatorSubsytem));
        operaterController.rightBumper().onTrue(new AgitatorOn(scoreSubsystem));
        operaterController.rightBumper().onTrue(new OutfeedRoller(intakeSubsystem));

        // operatorLeftTrigger.onTrue(new FloorIntakePosition(intakeSubsystem));
        // operaterController.leftBumper().onTrue(new LoadingIntakePosition(intakeSubsystem));
        // operaterController.povLeft().onTrue(new ScoreLeftState(scoreSubsystem));
        // operaterController.povRight().onTrue(new ScoreRightState(scoreSubsystem));
        // operaterController.a().onTrue(new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.b().onTrue(new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.x().onTrue(new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.y().onTrue(new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        //hasCoral.onTrue(new HandoffScorePosition(scoreSubsystem));

        //Telemetry

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
