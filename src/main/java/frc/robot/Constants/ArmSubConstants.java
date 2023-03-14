// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Add your docs here. */
public class ArmSubConstants {
    public static final double MAX_ROTATION_SPEED = 0.3;

    //positions
    public static final double DEFAULT_PIVOT = 0.5;
    public static final double DEFAULT_TELESCOPE = 0.5;
    public static final double POS1_PIVOT = 29.6;
    public static final double POS1_TELESCOPE = 0.5;
    public static final double POS2_PIVOT = 32.1;
    public static final double POS2_TELESCOPE = 0.5;
    public static final double POS3_PIVOT = 63;
    public static final double POS3_TELESCOPE = 0.5;

    //PID
    public static final double PIVOT_KP = 0.05;
    public static final double PIVOT_KI = 0.01;
    public static final double PIVOT_KD = 0.008;
    public static final double PIVOT_LIMIT = 10;
}
