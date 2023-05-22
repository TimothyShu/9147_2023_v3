// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.GyroSubsystem;
import frc.robot.Variables.PIDVariables;
import frc.robot.Variables.SubsystemVariables;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */

  double dAngle;
  double CurrentAngle;
  double dt;
  DriveSubsystem driveSubsystem;
  public AutoBalance(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.Brakemode();
    SubsystemVariables.DriveMode = "Auto";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double LastAngle = SubsystemVariables.DriveAutoBalanceAngle;
    
    dt = Timer.getFPGATimestamp() - PIDVariables.lastTimestamp;

    CurrentAngle = GyroSubsystem.get_roll();
    
    dAngle = (CurrentAngle - LastAngle)/dt;

    //remove noise
    if (Math.abs(dAngle) < 30) {
      dAngle = 0;
    }

    if (CurrentAngle > 5) {
      if (dAngle > -240) {
        driveSubsystem.Arcadedrive(0.4 + CurrentAngle*0.4/30, 0);
      }
    }

    if (CurrentAngle < -5) {
      if (dAngle < 240) {
        driveSubsystem.Arcadedrive(-0.4 + CurrentAngle*0.4/30, 0);
      }
    }

    SubsystemVariables.DriveAutoBalanceAngle = CurrentAngle;
  }

  public void end(boolean interrupted) {
    SubsystemVariables.DriveMode = "Arcade";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //check if dangle and andgle are close to 1
    if (Math.abs(dAngle) < 50 && Math.abs(CurrentAngle) < 5) {
      return true;
    } else {
      return false;
    }
  }
}
