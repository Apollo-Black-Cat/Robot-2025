// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Intake.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeCommands {
  /** Creates a new IntakeCommands. */
  public static Command runIntake(Intake intake, boolean isOn) {
    return Commands.runEnd(
        () -> {
          intake.runIntake(isOn);
        },
        () -> {
          intake.stopMotors();
        });
  }

  public static Command runIntake(Intake intake, boolean isOn, double seconds) {
    return Commands.runEnd(
            () -> {
              intake.runIntake(isOn);
            },
            () -> {
              intake.stopMotors();
            })
        .withTimeout(seconds);
  }
}
