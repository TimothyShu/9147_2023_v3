// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.ArmSubsystem;
import frc.robot.Subsystems.DriveSubsystem;
import frc.robot.Subsystems.PneumaticGrabber;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoMidSide extends SequentialCommandGroup {
  /** Creates a new autoMidSide. */
  public autoMidSide(DriveSubsystem drivesubsystem, ArmSubsystem armSubsystem, PneumaticGrabber pneumaticGrabber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {drivesubsystem.Brakemode();}),
      new InstantCommand(() -> {pneumaticGrabber.Retract();}),
      new InstantCommand(() -> {armSubsystem.Position2();}),
      new SetTelescope(armSubsystem, 1, 1.8),
      new WaitCommand(2),
      new InstantCommand(() -> {pneumaticGrabber.Extend();}),
      new WaitCommand(0.5),
      new InstantCommand(() -> {armSubsystem.DefaultPosition();}),
      new SetTelescope(armSubsystem, -1, 1.8),
      new PreAutobalance(drivesubsystem),
      new AutoBalance(drivesubsystem)
    );
  }
}
