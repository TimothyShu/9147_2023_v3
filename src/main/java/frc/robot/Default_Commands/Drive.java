// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Default_Commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Variables.SubsystemVariables;
import frc.robot.auto.AutoBalance;

public class Drive extends CommandBase {
  /** Creates a new Drive. */
  DriveSubsystem driveSubsystem;
  double speed;
  double turn;
  double heading;
  Command Balance = new AutoBalance(driveSubsystem);
  SendableChooser<Command> DriveChooser = new SendableChooser<>();
  public Drive(DriveSubsystem driveSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
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
    switch (mode) {
      case "Arcade":
        driveSubsystem.Arcadedrive(speed, turn);
        break;
      case "Heading":
        driveSubsystem.gyro_drive(speed, heading);
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
