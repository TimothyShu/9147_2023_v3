// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Variables;

/** Add your docs here. */
public class TargetVariables {

    //this creates a bit a freedom when a position is set, allowing for a bit of operator control
    public static double BasePivotTarget = 0.5;
    public static double BaseTelescopeTarget = 0.5;
    public static double PivotTargetOffset = 0;
    public static double TelescopeTargetOffset = 0;
    public static double PivotTarget = BasePivotTarget + PivotTargetOffset;
    public static double TelescopeTarget = BaseTelescopeTarget + TelescopeTargetOffset;

    //this is for the gyro drive mechanism
    public static double AngleTarget = 0;
}
