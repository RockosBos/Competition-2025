// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HandOffIntakePos extends Command {
  private Elevator e_Elevator;
  /** Creates a new HandOffIntakePos. */
  public HandOffIntakePos(Elevator e_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.e_Elevator = e_Elevator;
    addRequirements(this.e_Elevator);
  }

  //

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //m_Intake.setMotorVoltage(Constants.0.0);
    //m_Intake.setTargetPostion(Constants.INTAKE_ROTATE_HANDOFF_POS);
    e_Elevator.setIntakeTargetPostion(Constants.INTAKE_ELEVATOR_FLOOR_INTAKE_POS);
    e_Elevator.setScoreTargetPosition(Constants.SCORE_ELEVATOR_INTAKE_POSITION);
    //m_Score.setClawTargetPostion(Constants.SCORE_CLAW_OPEN_POS);
    //m_Score.setRotateTargetPostion(Constants.SCORE_ROTATE_CENTER_POS);
    //m_Score.setPivotTargetPostion(Constants.SCORE_PIVOT_IN_POS);
    //m_Score.setAgitatorRollerVoltage(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
