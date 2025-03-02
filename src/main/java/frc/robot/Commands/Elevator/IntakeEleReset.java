// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.ControlState;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeEleReset extends Command {
  Elevator e_Elevator;
  Timer timer;
  /** Creates a new EleHandoffPos. */
  public IntakeEleReset(Elevator e_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.e_Elevator = e_Elevator;
    addRequirements(e_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    e_Elevator.setIntakeEleControlState(ControlState.DIRECT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    e_Elevator.setIntakeEleVoltage(-2.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    e_Elevator.setIntakeEleVoltage(0.0);
    e_Elevator.setIntakeEleControlState(ControlState.CLOSEDLOOP);
    e_Elevator.setIntakeTargetPostion(Constants.INTAKE_ELEVATOR_FLOOR_INTAKE_POS);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return e_Elevator.intakeElevatorSensor();
  }
}
