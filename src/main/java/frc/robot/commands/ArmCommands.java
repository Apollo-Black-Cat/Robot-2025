// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.Arm.ArmConstants.Angulador.maxVoltage;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Arm.Angulador.Angulador;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

/** Add your docs here. */
public class ArmCommands {
  private static double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1;

  private ArmCommands() {}

  /** put an angle in the arm * */
  public static Command setAngle(Angulador angulador, double angle) {
    return new Command() {
      @Override
      public void initialize() {
        angulador.runCloseLoop(angle);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /** Control the arm by voltage by the control* */
  public static Command controlArm(Angulador angulador, DoubleSupplier supplier) {
    return Commands.run(
        () -> {
          double voltage = MathUtil.applyDeadband(supplier.getAsDouble(), DEADBAND);
          angulador.runOpenLoop(voltage * maxVoltage);
          SmartDashboard.putNumber("voltajeManita", voltage * maxVoltage);
        },
        angulador);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Angulador angulador) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
                () -> {
                  double voltage = timer.get() * FF_RAMP_RATE;
                  angulador.runOpenLoop(voltage);
                  velocitySamples.add(angulador.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                angulador)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
