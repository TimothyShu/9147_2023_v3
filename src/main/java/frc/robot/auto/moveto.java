// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.MovetoConstants;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.GyroSubsystem;
import frc.robot.Subsystems.OdometrySubsystem;
import frc.robot.Variables.PIDVariables;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Variables.SubsystemVariables;

public class moveto extends CommandBase {
  DriveSubsystem driveSubsystem;
  double distance;
  double xpos;
  double ypos;
  /** Creates a new moveto. */
  public moveto(DriveSubsystem driveSubsystem, double Xpos, double Ypos) {
    this.driveSubsystem = driveSubsystem;
    this.xpos = Xpos;
    this.ypos = Ypos;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SubsystemVariables.DriveMode = "Auto";
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d currentpos = OdometrySubsystem.GetCurrPose();
    double currentY = currentpos.getY();
    double currentX = currentpos.getX();

    double ydiff = ypos - currentY;
    double xdiff = xpos - currentX;
    double heading = Math.toDegrees(Math.atan2(ydiff, xdiff));
    double distance = Math.sqrt(Math.pow(xdiff, 2) + Math.pow(ydiff, 2));
    double error = distance;
    SmartDashboard.putNumber("Heading", heading);
    SmartDashboard.putNumber("Distance", distance);
    if (heading > 0.5 || heading < -0.5) {
      double currentheading = GyroSubsystem.get_yaw();
      double target_heading = currentheading + heading;
      driveSubsystem.gyro_drive(0, target_heading);
      return;
    }


    //Angles and quadrants
    double lastTimestamp = PIDVariables.lastTimestamp;
    double errorsum = PIDVariables.MovetoErrorSum;
    double Lasterror = PIDVariables.MovetoLastError;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    //integration
    if (Math.abs(error)<MovetoConstants.LIMIT) {
      errorsum += error * dt;
    }

    //derivative

    double error_rate = (error - Lasterror) / dt;

    //updates variables
    PIDVariables.DriveHeadingErrorSum = errorsum;
    PIDVariables.DriveHeadingLastError = error;

    //combining all terms

    double movespeed = MovetoConstants.KP * error + MovetoConstants.KI * errorsum + MovetoConstants.KD * error_rate;
    SmartDashboard.putNumber("Move speed", movespeed);
    SmartDashboard.putNumber("Distance", distance);
    double currentheading = GyroSubsystem.get_yaw();
    double target_heading = currentheading + heading;
    driveSubsystem.gyro_drive(movespeed, target_heading);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SubsystemVariables.DriveMode = "Arcade";
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (distance < 0.05) {
      return true;
    }
    return false;
  }
}
