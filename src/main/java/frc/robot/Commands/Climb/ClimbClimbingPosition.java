// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climb;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ClimbClimbingPosition extends Command {
  Climb c_Climb;
  /** Creates a new ClimbInPosition. */
  public ClimbClimbingPosition(Climb c_Climb) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.c_Climb = c_Climb;
    addRequirements(c_Climb);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    c_Climb.setClimbTargetPosition(Constants.CLIMB_CLIMBING_POS);
    c_Climb.setServoClimbingToLAPos(Constants.SERVO_UNLOCKED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return c_Climb.areWeInLA();
  }
}
