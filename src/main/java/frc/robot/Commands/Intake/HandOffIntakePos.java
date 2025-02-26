// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Intake;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class HandOffIntakePos extends Command {
  private Intake i_Intake;
  /** Creates a new HandOffIntakePos. */
  public HandOffIntakePos(Intake i_Intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.i_Intake = i_Intake;
    addRequirements(i_Intake);
  }

  //

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Command HandoffIntakePosition Started");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    i_Intake.setTargetPostion(Constants.INTAKE_ROTATE_HANDOFF_POS);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Command HandoffIntakePosition Complete");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return i_Intake.inPosition();
  }
}
