// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DevicePorts;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private final static CANSparkMax ArmPivotMotor = new CANSparkMax(DevicePorts.ARM_PIVOT_MOTOR, MotorType.kBrushless);
  private final static RelativeEncoder ArmPivotencoder = ArmPivotMotor.getEncoder();
  private final static VictorSPX ArmTelescopeMotor = new VictorSPX(DevicePorts.ARM_TELESCOPE_MOTOR);

  public ArmSubsystem() {
    ArmPivotMotor.setInverted(false);
    ArmTelescopeMotor.setInverted(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
