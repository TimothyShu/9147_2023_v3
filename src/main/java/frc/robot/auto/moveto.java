// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.OdometrySubsystem;

public class moveto extends CommandBase {
  DriveSubsystem driveSubsystem;
  double distance;
  double xpos;
  double ypos;
  /** Creates a new moveto. */
  public moveto(DriveSubsystem driveSubsystem, double distance, double Xpos, double Ypos) {
    this.driveSubsystem = driveSubsystem;
    this.distance = distance;
    this.xpos = Xpos;
    this.ypos = Ypos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentpos = OdometrySubsystem.GetCurrPose();
    double currentY = currentpos.getY();
    double currentX = currentpos.getX();

    double ydiff = ypos - currentY;
    double xdiff = xpos - currentX;
    double heading = Math.toDegrees(Math.atan2(ydiff, xdiff));
    //Angles and quadrants
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
