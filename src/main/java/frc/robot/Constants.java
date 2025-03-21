// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class Drive {
    public static final double maxSpeedMetersPerSec = 4.25;
    public static final double trackWidth = 0.62;

    // Device CAN IDs
    public static final int pigeonCanId = 9;
    public static final int leftLeaderCanId = 2;
    public static final int leftFollowerCanId = 3;
    public static final int rightLeaderCanId = 4;
    public static final int rightFollowerCanId = 5;

    // Motor configuration
    public static final int currentLimit = 80;
    public static final double wheelRadiusMeters = Units.inchesToMeters(3);
    public static final double motorReduction = 5.95;
    public static final boolean leftInverted = false;
    public static final boolean rightInverted = true;
    public static final DCMotor gearbox = DCMotor.getNEO(2);

    // Velocity PID configuration
    public static final double realKp = 0.0; // 0.050655
    public static final double realKd = 0.0;
    public static final double realKs = 0.0; // 0.377329
    public static final double realKv = 0.0; // 0.11862

    public static final double simKp = 0.07;
    public static final double simKd = 0.0;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;

    // PathPlanner configuration
    public static final double robotMassKg = 35.088;
    public static final double robotMOI = 13.480;
    public static final double wheelCOF = 1.2;
    public static final RobotConfig ppConfig =
        new RobotConfig(
            robotMassKg,
            robotMOI,
            new ModuleConfig(
                wheelRadiusMeters,
                maxSpeedMetersPerSec,
                wheelCOF,
                gearbox.withReduction(motorReduction),
                currentLimit,
                2),
            trackWidth);
  }

  public static class Angulador {
    // Motors ID
    public static final int leftMotorId = 1;
    public static final int rightMotorId = 6;

    public static final boolean leftInverted = false;
    public static final boolean rightInverted = true;

    // Motor reductions
    public static final double motorReduction = 500.0;

    public static final double InitialAngle = 0.0;

    public static final int currentLimit = 60;

    public static final double maxVoltage = 10.0;

    // Velocity PID configuration
    public static final double realKp = 1.0;
    public static final double realKd = 0.0;
    public static final double realKs = 0.0;
    public static final double realKv = 0.1;

    public static final double simKp = 30;
    public static final double simKd = 0.0;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;

    // Arm configuration
    public static final double armLength = 0.5;
    public static final double armMass = 2.0;
    public static final double armMaxAngle = 180.0;
    public static final double armMinAngle = 0.0;
    public static final double armInitialAngle = 0.0;
    public static final double MOI = SingleJointedArmSim.estimateMOI(armLength, armMass);
    public static final double armEncoderDistancePerPulse = 2 * Math.PI / 4096.0;
    public static final double gearRadius = Units.inchesToMeters(0.6);

    // Robot positions
    public static final double proccessorPosition = -45;
    public static final double level1Position = -20;
    public static final double level2Position = 0;
    public static final double level3Position = 10;
    public static final double coralStationPosition = 75;
    public static final double climbPosition = 30;

    public static final double minPosition = -70;
    public static final double maxPosition = 80;
  }

  public static class Elevador {
    public static final int elevatorMotorId = 7;

    public static final double elevatorMotorMaxSpeed = 1.0;

    public static final int currentLimit = 60;

    public static final boolean isInverted = false;

    public static final double motorReduction = 21.0;
    public static final double carriageMass = 0.3;
    public static final double drumRadius = Units.inchesToMeters(0.944);
    public static final double minHeight = 0.75;
    public static final double maxHeight = 1.10;
    public static final double starningHeight = 0.75;
    public static final double distancePerPulse = 2 * Math.PI / 4096.0;

    public static final double realKp = 0.1;
    public static final double realKd = 0.0;
    public static final double realKs = 0.01;
    public static final double realKv = 0.1;
    public static final double realKa = 0.0;
    public static final double realKg = 0.0;

    public static final double simKp = 0.10;
    public static final double simKd = 0.0;
    public static final double simKs = 0.01;
    public static final double simKv = 0.227;
    public static final double simKa = 0.0;
    public static final double simKg = 0.0;
  }

  public static class Intake {
    public static final int smallRollerId = 9;
    public static final int algaeRollersId = 10;

    public static final boolean smallRollerInverted = true;
    public static final boolean algaeRollerInverted = false;

    public static final double speed = 1.0;
    public static final double maxVoltage = 12;
    public static final int CurrentLimit = 30;
    public static final double IntakeGearing = 60.0;
  }

  public static class Wrist {
    // Motors ID
    public static final int wristMotorId = 8;

    public static final boolean wristInverted = true;

    public static final boolean encoderInverted = true;

    // Motor reductions
    public static final double motorReduction = 1.0;

    public static final int currentLimit = 60;

    public static final double maxVoltage = 10.0;

    public static final double positionFactor = 2 * Math.PI;

    // Velocity PID configuration
    public static final double realKp = 0.5;
    public static final double realKd = 0.0;
    public static final double realKs = 0.0;
    public static final double realKv = 0.1;

    public static final double simKp = 1.5;
    public static final double simKd = 0.1;
    public static final double simKs = 0.0;
    public static final double simKv = 0.227;

    // Arm configuration
    public static final double intakeLength = 0.5;
    public static final double intakeMass = 2.0;
    public static final double intakeMaxAngle = 180.0;
    public static final double intakeMinAngle = 0.0;
    public static final double intakeInitialAngle = 0.0;
    public static final double MOI = SingleJointedArmSim.estimateMOI(intakeLength, intakeMass);
    public static final double armEncoderDistancePerPulse = 2 * Math.PI / 4096.0;
    public static final double gearRadius = Units.inchesToMeters(0.6);
  }

  public static class Leds {
    public static final int pwmPort = 0;
    public static final int ledBuffer = 36;
  }
}
