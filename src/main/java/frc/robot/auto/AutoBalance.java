// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.GyroSubsystem;
import frc.robot.Variables.SubsystemVariables;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  DriveSubsystem driveSubsystem;
  public AutoBalance(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.Brakemode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double LastAngle = SubsystemVariables.DriveAutoBalanceAngle;

    double CurrentAngle = GyroSubsystem.get_roll();

    double dAngle = CurrentAngle - LastAngle;

    //remove noise
    if (Math.abs(dAngle) < 1) {
      dAngle = 0;
    }

    if (CurrentAngle > 5) {
      if (dAngle > -2) {
        driveSubsystem.Arcadedrive(0.8, 0);
      }
    }

    if (CurrentAngle < -5) {
      if (dAngle < 2) {
        driveSubsystem.Arcadedrive(0.8, 0);
      }
    }
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
