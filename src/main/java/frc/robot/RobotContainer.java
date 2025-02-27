// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.RumbleController;
import frc.robot.Commands.RumbleCooldown;
import frc.robot.Commands.CommandGroups.Sequential.AutoIntakeLoading;
import frc.robot.Commands.CommandGroups.Sequential.Handoff;
import frc.robot.Commands.CommandGroups.Sequential.IntakeFloor;
import frc.robot.Commands.CommandGroups.Sequential.IntakeLoading;
import frc.robot.Commands.CommandGroups.Sequential.L1;
import frc.robot.Commands.CommandGroups.Sequential.L2;
import frc.robot.Commands.CommandGroups.Sequential.L3;
import frc.robot.Commands.CommandGroups.Sequential.L4;
import frc.robot.Commands.CommandGroups.Sequential.ScoreCoral;
import frc.robot.Commands.CommandGroups.Sequential.TipProtection;
import frc.robot.Commands.Drive.DriveToNearestLoading;
import frc.robot.Commands.Drive.DriveToNearestScore;
import frc.robot.Commands.Drive.UpdateCameras;
import frc.robot.Commands.Elevator.IntakeEleHandoffPos;
import frc.robot.Commands.Elevator.IntakeEleFloorPos;
import frc.robot.Commands.Elevator.ScoreEleL2Position;
import frc.robot.Commands.Elevator.ScoreEleL3Position;
import frc.robot.Commands.Elevator.ScoreEleL4Position;
import frc.robot.Commands.Elevator.IntakeEleLoadingPos;
import frc.robot.Commands.Intake.FloorIntakePosition;
import frc.robot.Commands.Intake.HandOffIntakePos;
import frc.robot.Commands.Intake.IntakeRollerOff;
import frc.robot.Commands.Intake.L1IntakePos;
import frc.robot.Commands.Intake.LoadingIntakePosition;
import frc.robot.Commands.Intake.OutfeedRoller;
import frc.robot.Commands.Score.AgitatorOn;
import frc.robot.Commands.Score.ClawClosed;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.ClawRelease;
import frc.robot.Commands.Score.HandoffScorePosition;
import frc.robot.Commands.Score.ScoreLeftState;
import frc.robot.Commands.Score.ScoreRightState;
import frc.robot.Commands.Score.ScoreSetCenter;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.enums.CameraType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.PoseHandler;
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
    private final PoseHandler PoseHandlerSubsystem = new PoseHandler();

    public final CameraSubsystem PhotonVisionCamera1 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 1", new Transform3d(0.0762, 0.252349, 0.0271018, new Rotation3d(0,0,0)), PoseHandlerSubsystem.getAprilTagFieldLayout());
    public final CameraSubsystem PhotonVisionCamera2 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 2", new Transform3d(0.0762, -0.252349, 0.0271018, new Rotation3d(0,0,0)), PoseHandlerSubsystem.getAprilTagFieldLayout());
    //public final CameraSubsystem PhotonVisionCamera1 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 1", new Transform3d(0, 0, -0, new Rotation3d(0,0,0)), PoseHandlerSubsystem.getAprilTagFieldLayout());
    //public final CameraSubsystem PhotonVisionCamera2 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 2", new Transform3d(0, 0, 0, new Rotation3d(0,0,0)), PoseHandlerSubsystem.getAprilTagFieldLayout());
    
    // public final CameraSubsystem LimelightCamera = new CameraSubsystem(CameraType.LIMELIGHT, "limelight");

    private boolean rumbleCooldown = false;

    //Triggers
    private final Trigger driverLeftTrigger = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3);
    private final Trigger driverRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);
    private final Trigger operaterLeftTrigger = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3);
    private final Trigger operaterRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);

    private final Trigger hasCoral = new Trigger(() -> intakeSubsystem.hasCoral() && !DriverStation.isAutonomous());
    private final Trigger hasCoralRumble = new Trigger(() -> intakeSubsystem.hasCoral() && !DriverStation.isAutonomous() && !rumbleCooldown);
    private final Trigger tipProtection = new Trigger(() -> 
                                            Math.abs(drivetrain.getPigeon2().getRoll().getValueAsDouble()) > Constants.TIP_PROTECTION_THRESHOLD_ROLL ||
                                            Math.abs(drivetrain.getPigeon2().getPitch().getValueAsDouble()) > Constants.TIP_PROTECTION_THRESHOLD_ROLL);

    //Chooser for Autonomous Modes
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        //Data Log Initialization
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        //Named Commands for Pathplanner
        NamedCommands.registerCommand("ScoreL1", new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("ScoreL2", new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("ScoreL3", new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("ScoreL4", new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("AutoIntakeLoading", new AutoIntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("Handoff", new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        //NamedCommands.registerCommand("AutoAlign", new Command());

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
                drive.withVelocityX(-driverController.getLeftY() * elevatorSubsytem.getDriveSpeedLimit() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverController.getLeftX() * elevatorSubsytem.getDriveSpeedLimit() * MaxSpeed) // Drive left with negative X (left)
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
        driverController.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(PoseHandlerSubsystem.getXController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(PoseHandlerSubsystem.getYController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(PoseHandlerSubsystem.getTController(drivetrain.getPigeon2().getYaw().getValueAsDouble()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        driverController.rightBumper().whileTrue(new DriveToNearestScore(drivetrain, PoseHandlerSubsystem));
        driverController.rightBumper().whileTrue(new UpdateCameras(PhotonVisionCamera1, PhotonVisionCamera2));
        driverController.leftBumper().whileTrue(new DriveToNearestLoading(drivetrain, PoseHandlerSubsystem));
        driverController.leftBumper().whileTrue(new UpdateCameras(PhotonVisionCamera1, PhotonVisionCamera2));

        //Operator Controller

        operaterController.povLeft().onTrue(new ScoreLeftState(scoreSubsystem));
        operaterController.povRight().onTrue(new ScoreRightState(scoreSubsystem));

        operaterLeftTrigger.onTrue(new IntakeFloor(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterController.leftBumper().onTrue(new IntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterController.a().onTrue(new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterController.b().onTrue(new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterController.x().onTrue(new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterController.y().onTrue(new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        operaterRightTrigger.onTrue(new ScoreCoral(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        operaterController.povDown().onTrue(new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        hasCoral.onTrue(new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        hasCoralRumble.onTrue(new RumbleController(driverController, 0.5));
        hasCoral.onFalse(new RumbleCooldown(rumbleCooldown));
        tipProtection.onTrue(new TipProtection(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        

        //Telemetry

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
