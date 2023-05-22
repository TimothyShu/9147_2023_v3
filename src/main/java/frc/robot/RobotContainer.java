// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Default_Commands.*;
import frc.robot.Subsystems.*;
import frc.robot.Variables.SubsystemVariables;
import frc.robot.auto.autoLongSide;
import frc.robot.auto.autoMidSide;
import frc.robot.auto.autoShortSide;
import frc.robot.auto.autoTest;
import frc.robot.auto.moveto;

public class RobotContainer {

  //declare subsystems


  XboxController joystick1 = new XboxController(0);

  Joystick joystick2 = new Joystick(1);

  ArmSubsystem armSubsystem = new ArmSubsystem();
  DriveSubsystem driveSubsystem = new DriveSubsystem();
  PneumaticGrabber pneumaticGrabber = new PneumaticGrabber();
  GyroSubsystem gyroSubsystem = new GyroSubsystem();
  OdometrySubsystem odometrySubsystem = new OdometrySubsystem(gyroSubsystem, driveSubsystem);

  //Chooser
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  //Auto Commands
  Command ShortSide = new autoShortSide(armSubsystem, driveSubsystem, pneumaticGrabber);
  Command LongSide = new autoLongSide(driveSubsystem, armSubsystem, pneumaticGrabber);
  Command Test = new autoTest(armSubsystem, driveSubsystem, pneumaticGrabber);
  Command Mid = new autoMidSide(driveSubsystem, armSubsystem, pneumaticGrabber);

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

  // __________________________
  //|                          |
  //|    RobotContainer        |
  //|__________________________|


  public RobotContainer() {
    configureBindings();
    m_chooser.addOption("Short Side", ShortSide);
    m_chooser.addOption("Long Side", LongSide);
    m_chooser.addOption("Test", Test);
    m_chooser.addOption("Mid", Mid);
    SmartDashboard.putData(m_chooser);
  }

  private void configureBindings() {


    //defaultcommands
    armSubsystem.setDefaultCommand(new Arm(armSubsystem, () -> getJoystickY(), () -> GetJoystickX()));

    driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, () -> joystick1.getLeftY(), () -> joystick1.getRightY(), () -> joystick1.getRightX()));


    Trigger leftTrigger = new JoystickButton(joystick1, 5);

    //right trigger on controller
    Trigger rightTrigger = new JoystickButton(joystick1, 6);
    //configure the buttons below

    Trigger ButtonA = new JoystickButton(joystick1, 1);
    //Trigger ButtonB = new JoystickButton(joystick1, 2);
    Trigger ButtonX = new JoystickButton(joystick1, 3);
    Trigger ButtonY = new JoystickButton(joystick1, 4);

    Trigger Trigger_front = new JoystickButton(joystick2, 1);
    Trigger Side_button = new JoystickButton(joystick2, 2);
    Trigger Button_left_down = new JoystickButton(joystick2, 3);
    Trigger Button_right_down = new JoystickButton(joystick2, 4);
    Trigger Button_left_up = new JoystickButton(joystick2, 5);
    Trigger Button_right_up = new JoystickButton(joystick2, 6);
    //Trigger Side_left_top = new JoystickButton(joystick2, 7);
    //Trigger Side_right_top = new JoystickButton(joystick2, 8);
    //Trigger Side_left_mid = new JoystickButton(joystick2, 9);
    //Trigger Side_right_mid = new JoystickButton(joystick2, 10);
    //Trigger Side_left_down = new JoystickButton(joystick2, 11);
    //Trigger Side_right_down = new JoystickButton(joystick2, 12);

    leftTrigger.onTrue(new InstantCommand(() -> {driveSubsystem.Coastmode();}));
    rightTrigger.onTrue(new InstantCommand(() -> {driveSubsystem.Brakemode();}));

    ButtonX.onTrue(ArcadeDrive);
    ButtonY.onTrue(HeadingDrive);
    ButtonA.onTrue(new moveto(driveSubsystem, 0, -1));
    //ButtonA.onTrue(AutoBalance);

    //Joystick
    Button_left_up.onTrue(new InstantCommand(() -> {armSubsystem.DefaultPosition();}));
    Button_left_down.onTrue(new InstantCommand(() -> {armSubsystem.Position1();}));
    Button_right_up.onTrue(new InstantCommand(() -> {armSubsystem.Position2();}));
    Button_right_down.onTrue(new InstantCommand(() -> {armSubsystem.Position3();}));
    Trigger_front.onTrue(new InstantCommand(() -> {pneumaticGrabber.Retract();}));
    Side_button.onTrue(new InstantCommand(() -> {pneumaticGrabber.Extend();}));
  }

  private double GetJoystickX () {
    return joystick2.getRawAxis(0);
  }

  private double getJoystickY () {
    return joystick2.getRawAxis(1);
  }

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
