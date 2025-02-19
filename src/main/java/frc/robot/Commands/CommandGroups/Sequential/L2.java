// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.EleL1Position;
import frc.robot.Commands.Elevator.EleL2Position;
import frc.robot.Commands.Intake.L1IntakePos;
import frc.robot.Commands.Score.AgitatorOff;
import frc.robot.Commands.Score.AgitatorOn;
import frc.robot.Commands.Score.ClawClosed;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.ClawRelease;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L2 extends SequentialCommandGroup {
  
  Elevator e_Elevator;
  Intake i_Intake;
  Score s_Score;
 
  public L2(Elevator e_Elevator, Intake i_Intake, Score s_Score) {

    this.e_Elevator = e_Elevator;
    this.i_Intake = i_Intake;
    this.s_Score = s_Score;

    addCommands(
      new Handoff(e_Elevator, i_Intake, s_Score),
      new ClawClosed(s_Score),
      new ParallelCommandGroup(new EleL2Position(e_Elevator), new AgitatorOn(s_Score)),
      new ParallelCommandGroup(new ScoreSetScore(s_Score), new AgitatorOff(s_Score))
    );
  }
}
