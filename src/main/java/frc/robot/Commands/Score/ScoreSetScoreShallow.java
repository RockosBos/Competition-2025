// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Score;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.ScoreState;
import frc.robot.subsystems.Score;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreSetScoreShallow extends Command {
  Score s_Score;
    /** Creates a new ScoreSetLeft. */
  public ScoreSetScoreShallow(Score s_Score) {
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
    if(s_Score.getScoreState() == ScoreState.LEFT){
      s_Score.setRotateTargetPostion(Constants.SCORE_ROTATE_LEFT_POS);
      s_Score.setPivotTargetPostion(Constants.SCORE_PIVOT_OUT_LEFT_SHALLOW_POS);
    }
    else{
      s_Score.setRotateTargetPostion(Constants.SCORE_ROTATE_RIGHT_POS);
      s_Score.setPivotTargetPostion(Constants.SCORE_PIVOT_OUT_RIGHT_SHALLOW_POS);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ScoreSetScore Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_Score.atPivotTarget() && s_Score.atRotateTarget());
  }
}
