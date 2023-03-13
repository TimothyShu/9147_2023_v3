// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.*;

public class Subsystems extends SubsystemBase {
  /** Creates a new Subsystems. */
  public DriveSubsystem driveSubsystem;
  public ArmSubsystem armSubsystem;
  public GrabberSubsystem grabberSubsystem;
  public VisionSubsystem visionSubsystem;
  public GyroSubsystem gyroSubsystem;
  public Subsystems() {
    driveSubsystem = new DriveSubsystem();
    armSubsystem = new DriveSubsystem();
    grabberSubsystem = new GrabberSubsystem();
    visionSubsystem = new VisionSubsystem();
    gyroSubsystem = new GyroSubsystem();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
