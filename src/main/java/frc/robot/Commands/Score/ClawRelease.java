// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Score;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Score;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClawRelease extends Command {
  Score s_Score;
  Timer timer = new Timer();
  /** Creates a new ClawClosed. */
  public ClawRelease(Score s_Score) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Score = s_Score;
    addRequirements(s_Score);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_Score.setClawTargetPostion(Constants.SCORE_CLAW_RELEASE_POS);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command ClawRelease Complete in " + timer.get() + " Seconds");
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.75){
      return true;
    }
    return false;
  }
}
