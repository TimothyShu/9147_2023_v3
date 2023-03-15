// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DevicePorts;

public class GrabberSubsystem extends SubsystemBase {

  private final static VictorSPX LeftServo = new VictorSPX(DevicePorts.GRABBER_LEFT_SERVO);
  private final static VictorSPX RightServo = new VictorSPX(DevicePorts.GRABBER_RIGHT_SERVO);
  /** Creates a new GrabberSubsystem. */
  public GrabberSubsystem() {
    LeftServo.enableVoltageCompensation(true);
    RightServo.enableVoltageCompensation(true);
  }

  public void MotorStrength(double force) {
    LeftServo.set(VictorSPXControlMode.PercentOutput, force);
    RightServo.set(VictorSPXControlMode.PercentOutput, force);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
