// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeEleL1Failop extends Command {
  Elevator e_Elevator;
  /** Creates a new ScoreL2Position. */
  public IntakeEleL1Failop(Elevator e_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.e_Elevator = e_Elevator;
    addRequirements(e_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    e_Elevator.setIntakeTargetPostion(Constants.INTAKE_ELEVATOR_L1_FAILOP_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Intake Elevator L1 Failop Command Completed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return e_Elevator.intakeEleInPosition();
  }
}
