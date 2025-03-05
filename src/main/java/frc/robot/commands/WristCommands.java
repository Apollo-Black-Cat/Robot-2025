// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Wrist.maxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Arm.Wrist.Wrist;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class WristCommands {
  private static double DEADBAND = 0.1;
  // private static final double FF_RAMP_RATE = 0.1;

  public WristCommands() {}

  public static Command setAngle(Wrist wrist, double angle) {
    return new RunCommand(
            () -> {
              wrist.runCloseLoop(angle);
            },
            wrist)
        .onlyWhile(() -> Math.abs(wrist.getAngle() - angle) > 0.1);
  }

  public static Command controlWrist(Wrist wrist, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          double voltage = MathUtil.applyDeadband(supplier.getAsDouble(), DEADBAND);
          wrist.runOpenLoop(voltage * maxVoltage);
        },
        wrist);
  }
}
