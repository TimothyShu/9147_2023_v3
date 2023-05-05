// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Variables.TargetVariables;

public class extendTelescope extends CommandBase {
  private ArmSubsystem armSubsystem;
  private double time;
  private double starttime;
  /** Creates a new extendTelescope. */
  public extendTelescope(ArmSubsystem armSubsystem, double time) {
    this.armSubsystem = armSubsystem;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.starttime = Timer.getFPGATimestamp();
    TargetVariables.telescopeauto = true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = 0.5;
    armSubsystem.set_extension_speed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.set_extension_speed(0);
    TargetVariables.telescopeauto = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double elapsetime = Timer.getFPGATimestamp() - starttime;
    if (elapsetime >= time) {
      return true;
    } else {
      return false;
    }
  }
}
