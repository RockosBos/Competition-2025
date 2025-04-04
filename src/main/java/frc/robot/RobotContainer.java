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
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.RumbleController;
import frc.robot.Commands.RumbleCooldown;
import frc.robot.Commands.Climb.ClimbCapturePosition;
import frc.robot.Commands.Climb.ClimbClimbingPosition;
import frc.robot.Commands.Climb.ClimbInPosition;
import frc.robot.Commands.Climb.ClimbLockServo;
import frc.robot.Commands.CommandGroups.Sequential.AutoIntakeLoading;
import frc.robot.Commands.CommandGroups.Sequential.AutonomousAutoAlign;
import frc.robot.Commands.CommandGroups.Sequential.ClimbCapture;
import frc.robot.Commands.CommandGroups.Sequential.ClimbingPosition;
import frc.robot.Commands.CommandGroups.Sequential.FailOpFloor;
import frc.robot.Commands.CommandGroups.Sequential.FailOpL1;
import frc.robot.Commands.CommandGroups.Sequential.Handoff;
import frc.robot.Commands.CommandGroups.Sequential.IntakeFloor;
import frc.robot.Commands.CommandGroups.Sequential.IntakeLoading;
import frc.robot.Commands.CommandGroups.Sequential.L1;
import frc.robot.Commands.CommandGroups.Sequential.L2;
import frc.robot.Commands.CommandGroups.Sequential.L3;
import frc.robot.Commands.CommandGroups.Sequential.L4;
import frc.robot.Commands.CommandGroups.Sequential.RemoveHighAlgae;
import frc.robot.Commands.CommandGroups.Sequential.ResetElevators;
import frc.robot.Commands.CommandGroups.Sequential.ScoreCoral;
import frc.robot.Commands.CommandGroups.Sequential.SetScoreLeftandUpdate;
import frc.robot.Commands.CommandGroups.Sequential.SetScoreRightandUpdate;
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
import frc.robot.Commands.LED.SetLED;
import frc.robot.Commands.Score.AgitatorOn;
import frc.robot.Commands.Score.ClawClosed;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.ClawRelease;
import frc.robot.Commands.Score.FailOpDisable;
import frc.robot.Commands.Score.FailOpEnable;
import frc.robot.Commands.Score.HandoffScorePosition;
import frc.robot.Commands.Score.ScoreLeftState;
import frc.robot.Commands.Score.ScoreRightState;
import frc.robot.Commands.Score.ScoreSetCenter;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.enums.CameraType;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ControllerInputSubsystem;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
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
    private final Climb climbSubsytem = new Climb();
    private final PoseHandler PoseHandlerSubsystem = new PoseHandler();
    private final ControllerInputSubsystem driveControllerModified = new ControllerInputSubsystem(driverController);
    private final LED ledSubsystem = new LED(Constants.LED_ID, Constants.LED_SIZE);

    //public final CameraSubsystem PhotonVisionCamera1 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 1", new Transform3d(0.3048, 0.29845, 0.2159, new Rotation3d(0,0, Math.toRadians(-26.0))), PoseHandlerSubsystem.getAprilTagFieldLayout());
    public final CameraSubsystem PhotonVisionCamera1 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 1", new Transform3d(0.2794 - 0.1, 0.2794 - 0.1, 0.1793875, new Rotation3d(0,Math.toRadians(15.0), Math.toRadians(-26.0))), PoseHandlerSubsystem.getAprilTagFieldLayout());
    public final CameraSubsystem PhotonVisionCamera2 = new CameraSubsystem(CameraType.PHOTONVISION, "PhotonVision Camera 2", new Transform3d(0.2794, -0.2794, 0.1793875, new Rotation3d(0,Math.toRadians(15.0), Math.toRadians(30.0))), PoseHandlerSubsystem.getAprilTagFieldLayout());

    
    // public final CameraSubsystem LimelightCamera = new CameraSubsystem(CameraType.LIMELIGHT, "limelight");

    private boolean rumbleCooldown = false;

    //Triggers
    private final Trigger driverLeftTrigger = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3);
    private final Trigger driverRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);
    private final Trigger operaterLeftTrigger = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3);
    private final Trigger operaterRightTrigger = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3);

    private final Trigger hasCoral = new Trigger(() -> intakeSubsystem.hasCoral() && !DriverStation.isAutonomous() && !scoreSubsystem.inFailOp());
    private final Trigger hasCoralRumble = new Trigger(() -> intakeSubsystem.hasCoral() && !DriverStation.isAutonomous() && !rumbleCooldown);
    private final Trigger tipProtection = new Trigger(() -> 
                                            Math.abs(drivetrain.getPigeon2().getRoll().getValueAsDouble()) > Constants.TIP_PROTECTION_THRESHOLD_ROLL ||
                                            Math.abs(drivetrain.getPigeon2().getPitch().getValueAsDouble()) > Constants.TIP_PROTECTION_THRESHOLD_ROLL);

    //private final Trigger disabledBluealliance = new Trigger(() -> DriverStation.isDisabled() && DriverStation.getAlliance().get() == Alliance.Blue);
    //private final Trigger disabledRedalliance = new Trigger(() -> DriverStation.isDisabled() && DriverStation.getAlliance().get() == Alliance.Red);
    //Chooser for Autonomous Modes

    private final Trigger normalPOVLeft = new Trigger(() -> operaterController.povLeft().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalPOVUp = new Trigger(() -> operaterController.povUp().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalPOVRight = new Trigger(() -> operaterController.povRight().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalPOVDown = new Trigger(() -> operaterController.povDown().getAsBoolean() && !scoreSubsystem.inFailOp());

    private final Trigger normalA = new Trigger(() -> operaterController.a().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalB = new Trigger(() -> operaterController.b().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalY = new Trigger(() -> operaterController.y().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalX = new Trigger(() -> operaterController.x().getAsBoolean() && !scoreSubsystem.inFailOp());

    private final Trigger normalLB = new Trigger(() -> operaterController.leftBumper().getAsBoolean() && !scoreSubsystem.inFailOp());
    private final Trigger normalRB = new Trigger(() -> operaterController.rightBumper().getAsBoolean() && !scoreSubsystem.inFailOp());

    private final Trigger normalLT = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3 && !scoreSubsystem.inFailOp());
    private final Trigger normalRT = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3 && !scoreSubsystem.inFailOp());

    private final Trigger failopLT = new Trigger(() -> operaterController.getLeftTriggerAxis() > 0.3 && scoreSubsystem.inFailOp());
    private final Trigger failopA = new Trigger(() -> operaterController.a().getAsBoolean() && scoreSubsystem.inFailOp());
    private final Trigger failopRT = new Trigger(() -> operaterController.getRightTriggerAxis() > 0.3 && scoreSubsystem.inFailOp());

    private final Trigger failOpEnabled = new Trigger(() -> scoreSubsystem.inFailOp());

    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {
        //Data Log Initialization
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());

        //Named Commands for Pathplanner
        NamedCommands.registerCommand("L1", new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("L2", new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("L3", new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("L4", new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("AutoIntakeLoading", new AutoIntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("Handoff", new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("ScoreCoral", new ScoreCoral(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("AutoAlign", new AutonomousAutoAlign(drivetrain, PoseHandlerSubsystem));
        NamedCommands.registerCommand("SetScoreLeft", new ScoreLeftState(scoreSubsystem));
        NamedCommands.registerCommand("SetScoreRight", new ScoreRightState(scoreSubsystem));
        NamedCommands.registerCommand("ReleaseClaw", new ClawRelease(scoreSubsystem));
        NamedCommands.registerCommand("OpenClaw", new ClawOpened(scoreSubsystem));
        NamedCommands.registerCommand("RemoveHighAlgae", new RemoveHighAlgae(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("EnableFailOp", new FailOpEnable(scoreSubsystem));
        NamedCommands.registerCommand("FailOpL1", new FailOpL1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        NamedCommands.registerCommand("FailOpOutfeed", new OutfeedRoller(intakeSubsystem));

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
                drive.withVelocityX(-driveControllerModified.getAllianceDirectedYDrive() * elevatorSubsytem.getDriveSpeedLimit() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driveControllerModified.getAllianceDirectedXDrive() * elevatorSubsytem.getDriveSpeedLimit() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverController.getLeftY(), -driverController.getLeftX()))
        // ));

        //driverController.a().onTrue(new ClimbInPosition(climbSubsytem));
        driverController.b().onTrue(new ClimbClimbingPosition(climbSubsytem));
        driverController.a().onTrue(new ClimbLockServo(climbSubsytem));
        driverController.y().onTrue(new ClimbCapture(elevatorSubsytem, intakeSubsystem, scoreSubsystem, climbSubsytem));

        // driverController.back().and(driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // driverController.back().and(driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // driverController.start().and(driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // driverController.start().and(driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        driverController.rightBumper().whileTrue(
            drivetrain.applyRequest(() ->
            drive.withVelocityX(PoseHandlerSubsystem.getXController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(PoseHandlerSubsystem.getYController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive left with negative X (left)
                .withRotationalRate(PoseHandlerSubsystem.getTController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get().getRotation().getDegrees()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        // driverController.leftBumper().whileTrue(
        //     drivetrain.applyRequest(() ->
        //     drive.withVelocityX(PoseHandlerSubsystem.getXController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive forward with negative Y (forward)
        //         .withVelocityY(PoseHandlerSubsystem.getYController(drivetrain.samplePoseAt(Utils.getCurrentTimeSeconds()).get()) * MaxSpeed) // Drive left with negative X (left)
        //         .withRotationalRate(PoseHandlerSubsystem.getTController(drivetrain.getPigeon2().getYaw().getValueAsDouble()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // // );
        driverController.rightBumper().whileTrue(new DriveToNearestScore(drivetrain, PoseHandlerSubsystem));
        driverController.rightBumper().whileTrue(new UpdateCameras(PhotonVisionCamera1, PhotonVisionCamera2));
        driverController.povUp().onTrue(new FailOpDisable(scoreSubsystem));
        driverController.povDown().onTrue(new FailOpEnable(scoreSubsystem));

        //Operator Controller

        normalPOVLeft.onTrue(new ScoreLeftState(scoreSubsystem));
        normalPOVRight.onTrue(new ScoreRightState(scoreSubsystem));
        normalPOVDown.onTrue(new ResetElevators(elevatorSubsytem, intakeSubsystem));

        normalLT.onTrue(new IntakeFloor(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalLB.onTrue(new IntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalA.onTrue(new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalB.onTrue(new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalX.onTrue(new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalY.onTrue(new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        normalRT.onTrue(new ScoreCoral(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        normalRB.onTrue(new RemoveHighAlgae(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        normalPOVUp.onTrue(new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        failopA.onTrue(new FailOpL1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        failopLT.onTrue(new FailOpFloor(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        failopRT.onTrue(new OutfeedRoller(intakeSubsystem));

        // operaterController.povLeft().onTrue(new ScoreLeftState(scoreSubsystem));
        // operaterController.povRight().onTrue(new ScoreRightState(scoreSubsystem));
        // operaterController.povDown().onTrue(new ResetElevators(elevatorSubsytem, intakeSubsystem));

        // operaterLeftTrigger.onTrue(new IntakeFloor(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.leftBumper().onTrue(new IntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.a().onTrue(new L1(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.b().onTrue(new L2(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.x().onTrue(new L3(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterController.y().onTrue(new L4(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        // operaterRightTrigger.onTrue(new ScoreCoral(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        // operaterController.rightBumper().onTrue(new RemoveHighAlgae(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        // operaterController.povUp().onTrue(new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        hasCoral.onTrue(new Handoff(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        hasCoral.onTrue(new SetLED(ledSubsystem, LEDPattern.gradient(GradientType.kDiscontinuous, Color.kGreen), Color.kGreen));
        hasCoralRumble.onTrue(new RumbleController(driverController, 0.5));
        hasCoral.onFalse(new RumbleCooldown(rumbleCooldown));
        hasCoral.onFalse(new SetLED(ledSubsystem, LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue), Color.kGreen));
        tipProtection.onTrue(new TipProtection(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        failOpEnabled.whileTrue(new SetLED(ledSubsystem, LEDPattern.gradient(GradientType.kDiscontinuous, Color.kRed), Color.kRed));
        failOpEnabled.onTrue(new FailOpFloor(elevatorSubsytem, intakeSubsystem, scoreSubsystem));
        failOpEnabled.onFalse(new SetLED(ledSubsystem, LEDPattern.gradient(GradientType.kDiscontinuous, Color.kBlue), Color.kBlue));
        failOpEnabled.onFalse(new IntakeLoading(elevatorSubsytem, intakeSubsystem, scoreSubsystem));

        //Telemetry

        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
