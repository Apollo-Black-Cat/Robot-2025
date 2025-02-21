// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator.ElevatorConstants;
import frc.robot.subsystems.Intake.Wrist.Wrist;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class WristCommands {
  private static double DEADBAND = 0.1;

  public static Command moveWrist(Wrist wrist, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          double voltage = MathUtil.applyDeadband(supplier.getAsDouble(), DEADBAND);
          wrist.runOpenLoop(voltage * ElevatorConstants.maxVoltage);
          SmartDashboard.putNumber("voltajeManita", voltage * ElevatorConstants.maxVoltage);
        },
        wrist);
  }

  public static Command setAngle(Wrist wrist, double angle) {
    return new Command() {
      @Override
      public void initialize() {
        wrist.runCloseLoop(angle);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
