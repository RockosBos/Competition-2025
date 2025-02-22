// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.enums.ControlState;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ScoreEleReset extends Command {
  Elevator e_Elevator;
  Timer timer;
  /** Creates a new EleHandoffPos. */
  public ScoreEleReset(Elevator e_Elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.e_Elevator = e_Elevator;
    addRequirements(e_Elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    e_Elevator.setScoreEleControlState(ControlState.DIRECT);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    e_Elevator.setScoreEleVoltage(-2.0);
    if(e_Elevator.getScoreEleCurrent() > 15){
      timer.start();
    }
    else{
      timer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    e_Elevator.resetScoreEle();
    e_Elevator.setScoreEleControlState(ControlState.CLOSEDLOOP);
    e_Elevator.setScoreTargetPosition(Constants.SCORE_ELEVATOR_GO_AWAY_POSITION);
    System.out.println("Score Elevator Reset Command Completed");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > 0.5){
      return true;
    }
    return false;
  }
}
