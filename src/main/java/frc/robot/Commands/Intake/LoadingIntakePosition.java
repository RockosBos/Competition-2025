// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LoadingIntakePosition extends Command {
  Intake i_Intake;
  /** Creates a new LoadingIntakePosition. */
  public LoadingIntakePosition(Intake intakeSubsystem) {
    this.i_Intake = intakeSubsystem;
    addRequirements(i_Intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_Intake.setTargetPostion(Constants.INTAKE_ROTATE_LOADING_INTAKE_POS);
    //i_Intake.setMotorVoltage(Constants.INTAKE_ROLLER_INFEED_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command LoadingIntakePosition Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i_Intake.inPosition();
  }
}
