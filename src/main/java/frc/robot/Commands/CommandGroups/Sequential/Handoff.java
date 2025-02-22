// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.IntakeEleHandoffPos;
import frc.robot.Commands.Elevator.ScoreEleHandoffPos;
import frc.robot.Commands.Intake.HandOffIntakePos;
import frc.robot.Commands.Intake.IntakeRollerOff;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.HandoffScorePosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Handoff extends SequentialCommandGroup {

  Elevator e_Elevator;
  Intake i_Intake;
  Score s_Score;
  /** Creates a new Handoff. */
  public Handoff(Elevator e_Elevator, Intake i_Intake, Score s_Score) {

    this.e_Elevator = e_Elevator;
    this.i_Intake = i_Intake;
    this.s_Score = s_Score;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ClawOpened(s_Score),
      new ParallelCommandGroup(new HandOffIntakePos(i_Intake), new HandoffScorePosition(s_Score)),
      new IntakeRollerOff(i_Intake),
      new IntakeEleHandoffPos(e_Elevator),
      new ScoreEleHandoffPos(e_Elevator)
    );
  }
}
