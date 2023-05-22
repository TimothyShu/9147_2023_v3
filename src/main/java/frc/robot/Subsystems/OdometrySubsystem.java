// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OdomoetryConstants;

public class OdometrySubsystem extends SubsystemBase {
  /** Creates a new OdometrySubsystem. */
  GyroSubsystem gyroSubsystem;
  DriveSubsystem driveSubsystem;
  static Pose2d CurrentPose;
  DifferentialDriveOdometry Odometry;
  List<Double> EncoderVal;
  Rotation2d GyroAngle;
  public OdometrySubsystem(GyroSubsystem gyroSubsystem, DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.gyroSubsystem = gyroSubsystem;
    EncoderVal = driveSubsystem.GetEncoderValues();
    CurrentPose = OdomoetryConstants.CurrPose2d;
    Odometry = new DifferentialDriveOdometry(gyroSubsystem.GetRotation2d(), EncoderVal.get(0), EncoderVal.get(1), CurrentPose);
  }

  public static Pose2d GetCurrPose() {
    return CurrentPose;
  }

  public double GetAngle() {
    return GetCurrPose().getRotation().getDegrees();
  }

  public double getX() {
    return GetCurrPose().getX();
  }

  public double getY() {
    return GetCurrPose().getY();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    GyroAngle = gyroSubsystem.GetRotation2d();
    EncoderVal = driveSubsystem.GetEncoderValues();
    CurrentPose = Odometry.update(GyroAngle, EncoderVal.get(0), EncoderVal.get(1));
    SmartDashboard.putNumber("Current rotation", GetAngle());
    SmartDashboard.putNumber("Xpox", getX());
    SmartDashboard.putNumber("Ypos", getY());
    //SmartDashboard.putData(CurrentPose);
  }
}
