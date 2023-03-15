// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DevicePorts;
import frc.robot.Constants.ArmSubConstants;
import frc.robot.Variables.PIDVariables;
import frc.robot.Variables.SubsystemVariables;
import frc.robot.Variables.TargetVariables;

public class ArmSubsystem extends SubsystemBase {
  /** Creates a new ArmSubsystem. */

  private final static CANSparkMax ArmPivotMotor = new CANSparkMax(DevicePorts.ARM_PIVOT_MOTOR, MotorType.kBrushless);
  private final static RelativeEncoder ArmPivotencoder = ArmPivotMotor.getEncoder();
  private final static VictorSPX ArmTelescopeMotor = new VictorSPX(DevicePorts.ARM_TELESCOPE_MOTOR);

  public ArmSubsystem() {
    ArmPivotMotor.setInverted(false);
    ArmTelescopeMotor.setInverted(false);
    DefaultPosition();
  }

  public void move_to_position() {

    double PivotTarget = TargetVariables.PivotTarget;
    //double TelescopeTarget = TargetVariables.TelescopeTarget;

    double dT = Timer.getFPGATimestamp() - PIDVariables.lastTimestamp;

    //rotational PID

    //either use gyroscope or encoder
    double ArmPivotError = PivotTarget - ArmPivotencoder.getPosition();
    double ArmPivotErrorSum = PIDVariables.ArmPivotErrorSum;
    double ArmPivotLastError = PIDVariables.ArmPivotLastError;

    if (Math.abs(ArmPivotError)<ArmSubConstants.PIVOT_LIMIT) {
      ArmPivotErrorSum += ArmPivotError * dT;
    }

    double ArmPivotError_rate = (ArmPivotError - ArmPivotLastError) / dT;

    PIDVariables.ArmPivotErrorSum = ArmPivotErrorSum;
    PIDVariables.ArmPivotLastError = ArmPivotError;

    double turnspeed = ArmSubConstants.PIVOT_KP * ArmPivotError + ArmSubConstants.PIVOT_KI * ArmPivotErrorSum + ArmSubConstants.PIVOT_KD * ArmPivotError_rate;

    set_turn_speed(turnspeed);
  }

  public void set_turn_speed(double speed) {
    ArmPivotMotor.set(speed * ArmSubConstants.MAX_ROTATION_SPEED);
    SmartDashboard.putNumber("Pivot turn speed", speed * ArmSubConstants.MAX_ROTATION_SPEED);
  }

  public void DefaultPosition() {
    TargetVariables.BasePivotTarget = ArmSubConstants.DEFAULT_PIVOT;
    TargetVariables.BaseTelescopeTarget = ArmSubConstants.DEFAULT_TELESCOPE;
    SubsystemVariables.ArmPos = "DefaultPos";
  }

  public void Position1() {
    TargetVariables.BasePivotTarget = ArmSubConstants.POS1_PIVOT;
    TargetVariables.BaseTelescopeTarget = ArmSubConstants.POS1_TELESCOPE;
    SubsystemVariables.ArmPos = "Pos1";
  }

  public void Position2 () {
    TargetVariables.BasePivotTarget = ArmSubConstants.POS2_PIVOT;
    TargetVariables.BaseTelescopeTarget = ArmSubConstants.POS2_TELESCOPE;
    SubsystemVariables.ArmPos = "Pos2";
  }

  public void Position3 () {
    TargetVariables.BasePivotTarget = ArmSubConstants.POS3_PIVOT;
    TargetVariables.BaseTelescopeTarget = ArmSubConstants.POS3_TELESCOPE;
    SubsystemVariables.ArmPos = "Pos3";
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
