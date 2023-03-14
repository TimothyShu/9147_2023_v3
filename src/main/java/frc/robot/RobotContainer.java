// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Default_Commands.*;
import frc.robot.Subsystems.*;
import frc.robot.Variables.SubsystemVariables;

public class RobotContainer {

  //declare subsystems

  SubsystemsAll Subsystems = new SubsystemsAll();

  XboxController joystick1 = new XboxController(0);

  Joystick joystick2 = new Joystick(1);

  ArmSubsystem armSubsystem = new ArmSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  GrabberSubsystem grabberSubsystem = new GrabberSubsystem();

  // __________________________
  //|                          |
  //|    Instant Commands      |
  //|__________________________|
  InstantCommand Brakemode = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      driveSubsystem.Brakemode();
    }
  }
  , driveSubsystem);

  InstantCommand Coastmode = new InstantCommand(new Runnable() {@Override
    public void run() {
      driveSubsystem.Coastmode();
    }
  }, driveSubsystem);

  InstantCommand ArcadeDrive = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      SubsystemVariables.DriveMode = "Arcade";
    }
  }, driveSubsystem);

  InstantCommand HeadingDrive = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      SubsystemVariables.DriveMode = "Heading";
    }
  }, driveSubsystem);

  InstantCommand AutoBalance = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      SubsystemVariables.DriveMode = "Balance";
    }
  }, driveSubsystem);

  InstantCommand DefaultPos = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      armSubsystem.DefaultPosition();
    }
  }, armSubsystem);

  InstantCommand Pos1 = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      armSubsystem.Position1();;
    }
  }, armSubsystem);
  
  InstantCommand Pos2 = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      armSubsystem.Position2();
    }
  }, armSubsystem);

  InstantCommand Pos3 = new InstantCommand(new Runnable() {
    @Override
    public void run() {
      armSubsystem.Position3();
    }
  }, armSubsystem);

  // __________________________
  //|                          |
  //|    RobotContainer        |
  //|__________________________|


  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {

    //defaultcommands
    armSubsystem.setDefaultCommand(new Arm(armSubsystem, () -> getJoystickY(), () -> GetJoystickX()));

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, () -> joystick1.getLeftY(), () -> getJoystickY(), () -> GetJoystickX()));


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

    leftTrigger.onTrue(Coastmode);
    rightTrigger.onTrue(Brakemode);

    ButtonX.onTrue(ArcadeDrive);
    ButtonY.onTrue(HeadingDrive);
    ButtonA.onTrue(AutoBalance);

    //Joystick
    Button_left_up.onTrue(DefaultPos);
    Button_left_down.onTrue(Pos1);
    Button_right_up.onTrue(Pos2);
    Button_right_down.onTrue(Pos3);
  }

  private double GetJoystickX () {
    return joystick2.getRawAxis(0);
  }

  private double getJoystickY () {
    return joystick2.getRawAxis(1);
  }




  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
