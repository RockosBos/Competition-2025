// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Elevator.IntakeEleFloorPos;
import frc.robot.Commands.Elevator.IntakeEleLoadingPos;
import frc.robot.Commands.Elevator.ScoreEleIdlePosition;
import frc.robot.Commands.Intake.FloorIntakePosition;
import frc.robot.Commands.Intake.IntakeRollerIn;
import frc.robot.Commands.Intake.IntakeWaitForCoral;
import frc.robot.Commands.Intake.LoadingIntakePosition;
import frc.robot.Commands.Score.ClawOpened;
import frc.robot.Commands.Score.ScoreSetCenter;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoIntakeLoading extends SequentialCommandGroup {
  
  Elevator e_Elevator;
  Intake i_Intake;
  Score s_Score;
 
  public AutoIntakeLoading(Elevator e_Elevator, Intake i_Intake, Score s_Score) {

    this.e_Elevator = e_Elevator;
    this.i_Intake = i_Intake;
    this.s_Score = s_Score;

    addCommands(
      new ClawOpened(s_Score),
      new ParallelCommandGroup(new ScoreEleIdlePosition(e_Elevator), new ScoreSetCenter(s_Score)),
      new LoadingIntakePosition(i_Intake),
      new IntakeEleLoadingPos(e_Elevator),
      new IntakeRollerIn(i_Intake),
      new IntakeWaitForCoral(i_Intake)
    );
  }
}
