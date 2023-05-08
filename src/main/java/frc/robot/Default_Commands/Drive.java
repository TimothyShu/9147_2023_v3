// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Default_Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Variables.SubsystemVariables;
import frc.robot.Variables.TargetVariables;
import frc.robot.auto.AutoBalance;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  DriveSubsystem driveSubsystem;
  double speed;
  double turn;
  double heading;
  Command Balance = new AutoBalance(driveSubsystem);
  DoubleSupplier LeftY, RightY, RightX;
  public Drive(DriveSubsystem driveSubsystem, DoubleSupplier LeftY, DoubleSupplier RightY, DoubleSupplier RightX) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.LeftY = LeftY;
    this.RightY = RightY;
    this.RightX = RightX;
    addRequirements(driveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    String mode = SubsystemVariables.DriveMode;
    speed = -LeftY.getAsDouble();
    turn = RightX.getAsDouble();
    heading = Math.toDegrees(Math.atan2(RightY.getAsDouble(), RightX.getAsDouble()));

    //if the user is not putting in a command, then defualt to the last target
    if (Math.sqrt(Math.pow(RightY.getAsDouble(), 2) + Math.pow(RightX.getAsDouble(), 2)) >= 0.5) {
      TargetVariables.AngleTarget = heading;
    }
    switch (mode) {
      case "Arcade":
        SmartDashboard.putString("drive mode", mode);
        driveSubsystem.Arcadedrive(speed, turn);
        break;
      case "Heading":
        SmartDashboard.putString("drive mode", mode);
        driveSubsystem.gyro_drive(speed, TargetVariables.AngleTarget);
        break;
      case "Balance":
        Balance.schedule();
        break;
      case "Auto":
        break;
    }
  }

  public void heading_drive() {
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
