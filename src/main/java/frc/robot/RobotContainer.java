// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  //declare subsystems

  SubsystemsAll Subsystems = new SubsystemsAll();

  XboxController joystick1 = new XboxController(0);

  Joystick joystick2 = new Joystick(1);


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    Trigger leftTrigger = new JoystickButton(joystick1, 5);

    //right trigger on controller
    Trigger rightTrigger = new JoystickButton(joystick1, 6);
    //configure the buttons below

    Trigger ButtonA = new JoystickButton(joystick1, 1);
    Trigger ButtonB = new JoystickButton(joystick1, 2);
    Trigger ButtonX = new JoystickButton(joystick1, 3);
    Trigger ButtonY = new JoystickButton(joystick1, 4);

    Trigger Trigger_front = new JoystickButton(joystick2, 1);
    Trigger Side_button = new JoystickButton(joystick2, 2);
    Trigger Button_left_down = new JoystickButton(joystick2, 3);
    Trigger Button_right_down = new JoystickButton(joystick2, 4);
    Trigger Button_left_up = new JoystickButton(joystick2, 5);
    Trigger Button_right_up = new JoystickButton(joystick2, 6);
    Trigger Side_left_top = new JoystickButton(joystick2, 7);
    Trigger Side_right_top = new JoystickButton(joystick2, 8);
    Trigger Side_left_mid = new JoystickButton(joystick2, 9);
    Trigger Side_right_mid = new JoystickButton(joystick2, 10);
    Trigger Side_left_down = new JoystickButton(joystick2, 11);
    Trigger Side_right_down = new JoystickButton(joystick2, 12);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
