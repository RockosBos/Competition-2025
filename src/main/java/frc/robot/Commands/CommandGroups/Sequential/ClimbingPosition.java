// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CommandGroups.Sequential;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ClimbClimbingPosition;
import frc.robot.Commands.ClimbLockServo;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Score;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbingPosition extends SequentialCommandGroup {
  /** Creates a new ClimbingPosition. */

  Elevator e_Elevator;
  Intake i_Intake;
  Score s_Score;
  Climb c_Climb;

  public ClimbingPosition(Elevator e_Elevator, Intake i_Intake, Score s_Score, Climb c_Climb) {

    this.e_Elevator = e_Elevator;
    this.i_Intake = i_Intake;
    this.s_Score = s_Score;
    this.c_Climb = c_Climb;

    addCommands(
      new ClimbClimbingPosition(c_Climb),
      new ClimbLockServo(c_Climb)
    );
  }
}
