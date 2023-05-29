// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//create the object linked to the LED lights -refer to the WPI lib docs
//create a series of methods/functions that allows us to switch the lights on,off and choose colours

public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  AddressableLED LED = new AddressableLED(0);
  AddressableLEDBuffer LED_Buffer = new AddressableLEDBuffer(30);

  public Lights() {
    LED.setLength(LED_Buffer.getLength());
    LED.setData(LED_Buffer);
    LED.start();
  }

  public void Set_Colour(Integer Red, Integer Green, Integer Blue) {
    for (var i = 0; i < LED_Buffer.getLength(); i++) {
      LED_Buffer.setRGB(i, Red, Green, Blue);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
