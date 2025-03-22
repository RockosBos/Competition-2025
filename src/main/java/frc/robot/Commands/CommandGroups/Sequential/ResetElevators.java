// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.IntakeEleReset;
import frc.robot.Commands.Elevator.ScoreEleReset;
import frc.robot.Commands.Intake.HandOffIntakePos;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetElevators extends SequentialCommandGroup {
  /** Creates a new ResetElevators. */
  Elevator e_Elevator;
  Intake i_Intake;
  public ResetElevators(Elevator e_Elevator, Intake i_Intake) {
    this.e_Elevator = e_Elevator;

    addCommands(
      new ParallelCommandGroup(new HandOffIntakePos(i_Intake), new IntakeEleReset(e_Elevator)),
      new ScoreEleReset(e_Elevator)
    );
  }
}
