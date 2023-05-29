// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.GyroSubsystem;
import frc.robot.Variables.SubsystemVariables;

public class PreAutobalance extends CommandBase {
  /** Creates a new PreAutobalance. */
  double angle;
  double heading;
  DriveSubsystem driveSubsystem;

  public PreAutobalance(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SubsystemVariables.DriveMode = "Auto";
    heading = GyroSubsystem.get_yaw();
    angle = GyroSubsystem.get_roll();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angle  = GyroSubsystem.get_roll();
    driveSubsystem.gyro_drive(0.3, heading);;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubsystemVariables.DriveMode = "Arcade";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putNumber("current angle", angle);
    if (angle > 10) {
      return true;
    }
    return false;
  }
}
