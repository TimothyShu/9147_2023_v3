// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.DevicePorts;
import frc.robot.Constants.DriveSubConstants;
import frc.robot.Variables.PIDVariables;


public class DriveSubsystem extends SubsystemBase {
  /** Creates a new DriveSubsystem. */

  double speed_limit = DriveSubConstants.SPEED_LIMIT;
  double rotation_limit = DriveSubConstants.ROTATION_LIMIT;

  //Right motors
  private final static CANSparkMax RightMotor1 = new CANSparkMax(DevicePorts.DRIVE_RIGHT_MOTOR_1, MotorType.kBrushed);
  private final static CANSparkMax RightMotor2 = new CANSparkMax(DevicePorts.DRIVE_RIGHT_MOTOR_2, MotorType.kBrushed);

  //Left motorrs
  private final static CANSparkMax LeftMotor1 = new CANSparkMax(DevicePorts.DRIVE_LEFT_MOTOR_1, MotorType.kBrushed);
  private final static CANSparkMax LeftMotor2 = new CANSparkMax(DevicePorts.DRIVE_LEFT_MOTOR_2, MotorType.kBrushed);

  //Motor Controller groups
  private final static MotorControllerGroup Right_speed_group = new MotorControllerGroup(RightMotor1, RightMotor2);
  private final static MotorControllerGroup Left_speed_group = new MotorControllerGroup(LeftMotor1, LeftMotor2);

  public DriveSubsystem() {
    Right_speed_group.setInverted(false);
    Left_speed_group.setInverted(true);
  }

  public void drive(double speed, double rotation) {

    speed = speed * speed_limit;
    rotation = rotation * rotation_limit;

    //custom arcade drive
    double Left_out = speed + rotation;
    double Right_out = speed - rotation;

    //sets the motors
    Left_speed_group.set(Left_out);
    Right_speed_group.set(Right_out);
  }

  public void tankdrive(double left, double right) {
    left = left * speed_limit;
    right = right * speed_limit;
    Left_speed_group.set(left);
    Right_speed_group.set(right);
  }

  public void gyro_drive(double speed, double heading) {
    //using both the heading and the speed

    //get the current gyro number
    double gyro_yaw = GyroSubsystem.get_yaw();

    //implementing a custom PID control system

    double error = heading - gyro_yaw;
    //this will optomise the path
    if (error > 180) {
      error -= 360; // this is reversed because now it will turn in the opposite direction
    }
    if (error <= -180) {
      error += 360; //same thing
    }

    double lastTimestamp = PIDVariables.lastTimestamp;
    double errorsum = PIDVariables.DriveHeadingErrorSum;
    double Lasterror = PIDVariables.DriveHeadingLastError;
    double dt = Timer.getFPGATimestamp() - lastTimestamp;

    //integration
    if (Math.abs(error)<DriveSubConstants.LIMIT) {
      errorsum += error * dt;
    }

    //derivative

    double error_rate = (error - Lasterror) / dt;

    //updates variables
    PIDVariables.DriveHeadingErrorSum = errorsum;
    PIDVariables.DriveHeadingLastError = error;

    //combining all terms

    double turnspeed = DriveSubConstants.KP * error + DriveSubConstants.KI * errorsum + DriveSubConstants.KD * error_rate;

    if (turnspeed > 1) {
      turnspeed = 1;
    } else {
      if (turnspeed < -1) {
        turnspeed = -1;
      }
    }
    //we set the motors to these values
    SmartDashboard.putNumber("Heading drive rotation output", turnspeed);
    SmartDashboard.putNumber("Heading drive speed output", speed);
    drive(speed, turnspeed);
  }

  public void Brakemode() {
    RightMotor1.setIdleMode(IdleMode.kBrake);
    RightMotor2.setIdleMode(IdleMode.kBrake);
    LeftMotor1.setIdleMode(IdleMode.kBrake);
    LeftMotor2.setIdleMode(IdleMode.kBrake);
  }

  public void Coastmode() {
    RightMotor1.setIdleMode(IdleMode.kCoast);
    RightMotor2.setIdleMode(IdleMode.kCoast);
    LeftMotor1.setIdleMode(IdleMode.kCoast);
    LeftMotor1.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
