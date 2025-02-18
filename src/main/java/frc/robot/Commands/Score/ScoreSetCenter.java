// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Score;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreSetCenter extends Command {
  Score s_Score;
    /** Creates a new ScoreSetLeft. */
  public ScoreSetCenter(Score s_Score) {
    this.s_Score = s_Score;
    addRequirements(this.s_Score);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Score.setRotateTargetPostion(Constants.SCORE_ROTATE_CENTER_POS);
    s_Score.setPivotTargetPostion(Constants.SCORE_PIVOT_IN_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
