// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Default_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Variables.TargetVariables;

public class Arm extends CommandBase {
  ArmSubsystem armSubsystem;
  DoubleSupplier JoystickY;
  DoubleSupplier JoystickX;
  /** Creates a new Arm. */
  public Arm(ArmSubsystem armSubsystem, DoubleSupplier JoystickY, DoubleSupplier JoystickX) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.armSubsystem = armSubsystem;
    this.JoystickY = JoystickY;
    this.JoystickX = JoystickX;
    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //this changes the values and allows for movement
    TargetVariables.PivotTargetOffset = JoystickX.getAsDouble() * 2;
    TargetVariables.TelescopeTargetOffset = JoystickY.getAsDouble();
    armSubsystem.move_to_position();
    armSubsystem.set_extension_speed(JoystickY.getAsDouble());
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
