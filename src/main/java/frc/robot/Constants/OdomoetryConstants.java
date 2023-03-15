// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class OdomoetryConstants {
    public static final Pose2d BLUEPOS1 = new Pose2d(13.5, 14.2, new Rotation2d(Math.toRadians(180)));
    public static Pose2d CurrPose2d = new Pose2d(0,0, new Rotation2d());
}
