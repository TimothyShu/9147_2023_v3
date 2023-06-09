// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.GyroSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Variables.SubsystemVariables;

public class AutoDrive extends CommandBase {
  /** Creates a new driveForward. */
  private DriveSubsystem drivesubsystem;
  private double time;
  private double starttime;
  private double speed;
  private double heading;
  public AutoDrive(DriveSubsystem drivesubsystem,double speed, double time) {
    this.drivesubsystem = drivesubsystem;
    this.speed = speed;
    this.time = time;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SubsystemVariables.DriveMode = "Auto";
    this.starttime = Timer.getFPGATimestamp();
    this.heading = GyroSubsystem.get_yaw();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivesubsystem.gyro_drive(speed, heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivesubsystem.Arcadedrive(0, 0);
    SubsystemVariables.DriveMode = "Arcade";
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
