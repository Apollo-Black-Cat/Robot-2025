// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  /** Creates a new Leds. */
  LedsIO ledsIO;

  LedsIOInputsAutoLogged inputs = new LedsIOInputsAutoLogged();

  public Leds(LedsIO ledsIO) {
    this.ledsIO = ledsIO;
  }

  public void setSolidRed() {
    ledsIO.setSolidRed();
  }

  public void setSolidGreen() {
    ledsIO.setSolidGreen();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
