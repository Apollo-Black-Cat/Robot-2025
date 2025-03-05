// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm.Wrist;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Wrist.gearRadius;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
// import frc.robot.Constants;
// import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  /** Creates a new Angulador. */
  WristIO wristIO;

  WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  // private final double kS = Constants.currentMode == Mode.SIM ? simKs : realKs;
  // private final double kV = Constants.currentMode == Mode.SIM ? simKv : realKv;
  private final SysIdRoutine sysId;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
    // configure SysID
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Wrist/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runOpenLoop(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    wristIO.updateInputs(inputs);
  }

  public void runOpenLoop(double voltage) {
    wristIO.setVoltage(voltage);
  }

  public void runCloseLoop(double angle) {
    wristIO.setAngle(angle);
  }

  public void stop() {
    wristIO.setVoltage(0);
  }

  public Command getSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command getSysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @AutoLogOutput
  public double getAngle() {
    return Units.radiansToDegrees(inputs.positionDeg);
  }

  @AutoLogOutput
  public double getVelocity() {
    return inputs.velocityRadPerSec * gearRadius;
  }

  public double getCharacterizationVelocity() {
    return inputs.velocityRadPerSec;
  }
}
