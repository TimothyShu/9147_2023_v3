// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Default_Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.GrabberSubsystem;
import frc.robot.Variables.SubsystemVariables;

public class Grabber extends CommandBase {
  GrabberSubsystem grabberSubsystem;
  /** Creates a new Grabber. */
  public Grabber(GrabberSubsystem grabberSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.grabberSubsystem = grabberSubsystem;
    addRequirements(grabberSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String mode = SubsystemVariables.GrabberMode;
    switch (mode) {
      case "Retract":
        grabberSubsystem.MotorStrength(-1);
        break;
      case "Extend":
        grabberSubsystem.MotorStrength(1);
        break;
      case "Stop":
        grabberSubsystem.MotorStrength(0);
        break;
    }
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
