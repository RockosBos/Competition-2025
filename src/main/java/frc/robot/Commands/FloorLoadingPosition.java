// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class FloorLoadingPosition extends Command {
  Intake m_Intake;
  Elevator m_Elevator;
  Score m_Score;
  /** Creates a new FloorLoadingPosition. */
  public FloorLoadingPosition(Intake m_Intake, Elevator m_Elevator, Score m_Score) {
    this.m_Intake = m_Intake;
    this.m_Elevator = m_Elevator;
    this.m_Score = m_Score;
    addRequirements(this.m_Intake, this.m_Elevator, this.m_Score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setMotorVoltage(Constants.IntakeRollerForwardVoltage);
    m_Intake.setTargetPostion(Constants.IntakeArmFloorPos);
    m_Elevator.setIntakeTargetPostion(Constants.IntakeElevatorFloorIntakePos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(m_Intake.inPosition()){

    }
    return false;
  }
}
