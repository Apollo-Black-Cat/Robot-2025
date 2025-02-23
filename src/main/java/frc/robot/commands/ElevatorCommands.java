// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Elevador.Elevador;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ElevatorCommands {
  private static double DEADBAND = 0.1;

  public static Command controlElevator(Elevador elevador, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          double voltage = MathUtil.applyDeadband(supplier.getAsDouble(), DEADBAND);
          elevador.runOpenLoop(voltage * 12);
          SmartDashboard.putNumber("voltajeManita", voltage * 12);
        },
        elevador);
  }

  public static Command setHeight(Elevador elevador, double height) {
    return new Command() {
      @Override
      public void initialize() {
        elevador.runCloseLoop(height);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }
}
