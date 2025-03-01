// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Score.ScoreLeftState;
import frc.robot.Commands.Score.ScoreSetScore;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetScoreLeftandUpdate extends SequentialCommandGroup {
  /** Creates a new SetScoreLeftandUpdate. */
  public SetScoreLeftandUpdate(Score s_Score) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ScoreLeftState(s_Score),
      new ScoreSetScore(s_Score)
    );
  }
}
