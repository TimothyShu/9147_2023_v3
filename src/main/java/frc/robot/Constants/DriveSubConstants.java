// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class DriveSubConstants {
    public static final double SPEED_LIMIT = 0.65;
    public static final double ROTATION_LIMIT = 0.34;

    //PID
    public static final double KP = 0.1;
    public static final double KI = 0.3;
    public static final double KD = 0.008;
    public static final double LIMIT = 10;

    //Encoder constants
    //this is the ration of:
    // Pulse 20 : 1 rotation 1 : 1 gearbox 1 : 1 wheel 1 : 0.48 circumfrance
    // 20       : 1            : 0.114       : 0.114     : 0.0549
    public static final double ENCODER_RATIO = 0.00274;
}
