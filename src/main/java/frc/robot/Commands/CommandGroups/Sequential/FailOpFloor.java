// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.IntakeEleFloorPos;
import frc.robot.Commands.Elevator.IntakeEleHandoffPos;
import frc.robot.Commands.Elevator.ScoreEleHandoffPos;
import frc.robot.Commands.Elevator.ScoreEleIdlePosition;
import frc.robot.Commands.Elevator.ScoreEleL2Position;
import frc.robot.Commands.Elevator.ScoreEleL4Position;
import frc.robot.Commands.Elevator.ScoreEleZero;
import frc.robot.Commands.Intake.FloorIntakePosition;
import frc.robot.Commands.Intake.IntakeIdle;
import frc.robot.Commands.Intake.IntakeRollerIn;
import frc.robot.Commands.Intake.IntakeRollerOff;
import frc.robot.Commands.Intake.L1IntakePos;
import frc.robot.Commands.Intake.OutfeedRollerHandoff;
import frc.robot.Commands.Score.AgitatorOff;
import frc.robot.Commands.Score.AgitatorOn;
import frc.robot.Commands.Score.ClawClosed;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.ClawRelease;
import frc.robot.Commands.Score.ScoreRightState;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FailOpFloor extends SequentialCommandGroup {
  
  Elevator e_Elevator;
  Intake i_Intake;
  Score s_Score;
 
  public FailOpFloor(Elevator e_Elevator, Intake i_Intake, Score s_Score) {

    this.e_Elevator = e_Elevator;
    this.i_Intake = i_Intake;
    this.s_Score = s_Score;

    addCommands(
      new AgitatorOff(s_Score),
      new IntakeRollerOff(i_Intake),
      new ParallelCommandGroup(new IntakeEleFloorPos(e_Elevator), new FloorIntakePosition(i_Intake)),
      new ScoreRightState(s_Score),
      new ScoreSetScore(s_Score),
      new IntakeRollerIn(i_Intake),
      new ScoreEleHandoffPos(e_Elevator)
    );
  }
}
