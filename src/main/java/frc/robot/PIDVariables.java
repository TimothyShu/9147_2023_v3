// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class PIDVariables {

    //Timestamp
    public static double lastTimestamp = 0;

    //Drive
    public static double DriveHeadingLastError = 0;
    public static double DriveHeadingErrorSum = 0;
    //Drive constants
    public static final double DRIVE_HEADING_KP = 0.1;
    public static final double DRIVE_HEADING_KI = 0.3;
    public static final double DRIVE_HEADING_KD = 0.008;
    public static final double DRIVE_HEADING_LIMIT = 10;

    //Pivot
    public static double ArmPivotLastError = 0;
    public static double ArmPivotErrorSum = 0;

    //Telescope
    public static double ArmTelescopeLastError = 0;
    public static double ArmTelescopeErrorSum = 0;

    public static void UpdateTime() {
        lastTimestamp = Timer.getFPGATimestamp();
    }

}
