// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.Constants;

public class LedsIOBuffer implements LedsIO {
  /** Creates a new LedsIOBuffer. */
  private final AddressableLED led = new AddressableLED(Constants.Leds.pwmPort);

  private final AddressableLEDBuffer buffer = new AddressableLEDBuffer(Constants.Leds.ledBuffer);

  public LedsIOBuffer() {
    led.setLength(buffer.getLength());

    LEDPattern.solid(Color.kGold).applyTo(buffer);

    led.setData(buffer);
    led.start();
  }

  @Override
  public void setSolidRed() {
    LEDPattern.solid(Color.kRed).applyTo(buffer);
    led.setData(buffer);
  }

  @Override
  public void setSolidGreen() {
    LEDPattern.solid(Color.kGreen).applyTo(buffer);
    led.setData(buffer);
  }

  @Override
  public void updateInputs(LedsIOInputs inputs) {}

  public void periodic() {}
}
