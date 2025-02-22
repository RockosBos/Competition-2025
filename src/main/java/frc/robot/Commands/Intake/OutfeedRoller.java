// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;
import frc.robot.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class OutfeedRoller extends Command {
  Intake m_Intake;
  /** Creates a new FloorLoadingPosition. */
  public OutfeedRoller(Intake intakeSubsystem) {
    //this.m_Intake = m_Intake;
    this.m_Intake = intakeSubsystem;
    //this.m_Score = m_Score;
    addRequirements(this.m_Intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_Intake.setMotorVoltage(Constants.INTAKE_ROLLER_OUTFEED_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command FloorIntakePosition Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return m_Intake.inPosition();
  }
}
