// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Variables.SubsystemVariables;

public class PneumaticGrabber extends SubsystemBase {

  //This is our compressor
  Compressor pcmCompressor = new Compressor(PneumaticsModuleType.REVPH);

  //This is our solenoid
  DoubleSolenoid grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, 14, 15);

  /** Creates a new PneumaticGrabber. */
  public PneumaticGrabber() {

    pcmCompressor.enableDigital();
    //dissable the compressor
    pcmCompressor.disable();
  }

  public void setGrabber(Value value) {
    grabberSolenoid.set(value);
  }

  public void disable() {
    pcmCompressor.disable();
  }

  public void Extend() {
    SubsystemVariables.GrabberMode = "Extend";
    setGrabber(Value.kForward);
  }

  public void Retract() {
    SubsystemVariables.GrabberMode = "Retract";
    setGrabber(Value.kReverse);
  }

  public void Off() {
    SubsystemVariables.GrabberMode = "Off";
    setGrabber(Value.kOff);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SubsystemVariables.CompressorEnabled = pcmCompressor.isEnabled();
    SubsystemVariables.CompressorCurrent = pcmCompressor.getCurrent();
    SubsystemVariables.PressureSwitch = pcmCompressor.getPressureSwitchValue();
    //show these values
    SmartDashboard.putBoolean("CompressorEnabled", pcmCompressor.isEnabled());
    SmartDashboard.putBoolean("Pressure Switch", pcmCompressor.getPressureSwitchValue());
    SmartDashboard.putNumber("Compressor Current", pcmCompressor.getCurrent());
    SmartDashboard.putString("Grabber Mode", SubsystemVariables.GrabberMode);
  }
}
