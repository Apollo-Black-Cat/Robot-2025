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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.WristCommands;
import frc.robot.subsystems.Arm.Angulador.Angulador;
import frc.robot.subsystems.Arm.Angulador.AnguladorIO;
import frc.robot.subsystems.Arm.Angulador.AnguladorIOSim;
import frc.robot.subsystems.Arm.Angulador.AnguladorIOSpark;
import frc.robot.subsystems.Arm.Elevador.Elevador;
import frc.robot.subsystems.Arm.Elevador.ElevadorIO;
import frc.robot.subsystems.Arm.Elevador.ElevadorIOSim;
import frc.robot.subsystems.Arm.Elevador.ElevadorIOTalonFX;
import frc.robot.subsystems.Arm.Intake.*;
import frc.robot.subsystems.Arm.Wrist.Wrist;
import frc.robot.subsystems.Arm.Wrist.WristIO;
import frc.robot.subsystems.Arm.Wrist.WristIOSim;
import frc.robot.subsystems.Arm.Wrist.WristIOSpark;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSpark;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Angulador angulador;
  private final Elevador elevador;
  private final Wrist wrist;
  // private final Vision vision;
  private final Intake intake;
  // private final Leds leds;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  PathPlannerPath path;
  PathConstraints constraints;
  Command command;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(new DriveIOSpark(), new GyroIONavX());
        angulador = new Angulador(new AnguladorIOSpark());
        elevador = new Elevador(new ElevadorIOTalonFX());
        wrist = new Wrist(new WristIOSpark());
        /*vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
            new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation)); */
        intake = new Intake(new IntakeIOSpark());
        // leds = new Leds(new LedsIOBuffer());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        angulador = new Angulador(new AnguladorIOSim());
        elevador = new Elevador(new ElevadorIOSim());
        wrist = new Wrist(new WristIOSim());
        /*vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(
                VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
            new VisionIOPhotonVisionSim(
                VisionConstants.camera1Name, VisionConstants.robotToCamera1, drive::getPose)); */
        intake = new Intake(new IntakeIOSim());
        // leds = new Leds(null);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        angulador = new Angulador(new AnguladorIO() {});
        elevador = new Elevador(new ElevadorIO() {});
        wrist = new Wrist(new WristIO() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        intake = new Intake(new IntakeIO() {});
        // leds = new Leds(null);
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    try {
      path = PathPlannerPath.fromPathFile("Recoger Coral");
    } catch (FileVersionException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
    constraints = new PathConstraints(2.5, 3.0, 300, 651);
    command = AutoBuilder.pathfindThenFollowPath(path, constraints);
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal arcade drive

    // First driver buttons
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightX()));
    angulador.setDefaultCommand(ArmCommands.controlArm(angulador, () -> -controller.getRightY()));
    // Configurar el trugger para mover el elevador.
    elevador.setDefaultCommand(
        ElevatorCommands.controlElevator(
            elevador, () -> (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())));
    // wrist.setDefaultCommand(WristCommands.controlWrist(wrist, () -> -controller.getLeftX() *
    // .20));
    // Botón para mover la muñeca alternando entre 90 y 0 grados.
    controller.rightStick().onTrue(WristCommands.setAngle(wrist, 180));
    controller.a().onTrue(WristCommands.setAngle(wrist, 90));
    controller.b().onTrue(WristCommands.setAngle(wrist, 0));
    // Boton para activar coral intake
    controller.y().whileTrue(IntakeCommands.runCoralIntake(intake, true));
    controller.x().whileTrue(IntakeCommands.runCoralIntake(intake, false));
    // Boton para activar algae intake
    controller.povUp().whileTrue(IntakeCommands.runAlgaeIntake(intake, true));
    controller.povLeft().whileTrue(IntakeCommands.runAlgaeIntake(intake, false));
    // Botón para mover el angulador con el valor del segundo driver.
    controller
        .leftBumper()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setAngle(angulador), WristCommands.setAngle(wrist)));

    // Second driver buttons
    // Processor
    controller2
        .a()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(-46), WristCommands.setTargetPose(0)));
    // L1
    controller2
        .b()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(-25), WristCommands.setTargetPose(0)));
    // L2
    controller2
        .x()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(1), WristCommands.setTargetPose(90)));
    // L3
    controller2
        .y()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(10), WristCommands.setTargetPose(90)));
    // Coral Station
    controller2
        .leftBumper()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(80), WristCommands.setTargetPose(180)));
    // Climb
    controller2
        .rightBumper()
        .onTrue(
            new ParallelCommandGroup(
                ArmCommands.setTargetPose(30), WristCommands.setTargetPose(180)));

    /* // Only first driver buttons
    // Processor
    controller.a().onTrue(ArmCommands.setAngle(angulador, 0));
    // L1
    controller.b().onTrue(ArmCommands.setAngle(angulador, 20));
    // L2
    controller.x().onTrue(ArmCommands.setAngle(angulador, 45));
    //  Coral Station
    controller.y().onTrue(ArmCommands.setAngle(angulador, 60));
    // Coral Station
    controller.leftBumper().onTrue(ArmCommands.setAngle(angulador, 140));
    // Climb
    controller.rightBumper().onTrue(ArmCommands.setAngle(angulador, 90));
    */
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
