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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.ArmCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Arm.Angulador.Angulador;
import frc.robot.subsystems.Arm.Angulador.AnguladorIO;
import frc.robot.subsystems.Arm.Angulador.AnguladorIOSim;
import frc.robot.subsystems.Arm.Angulador.AnguladorIOSpark;
import frc.robot.subsystems.Arm.Elevador.Elevador;
import frc.robot.subsystems.Arm.Elevador.ElevadorIO;
import frc.robot.subsystems.Arm.Elevador.ElevadorIOSim;
import frc.robot.subsystems.Arm.Elevador.ElevadorIOTalonFX;
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
  // private final Wrist wrist;
  // private final Vision vision;
  // private final Intake intake;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

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
        // wrist = new Wrist(new WristIOSpark());
        /*vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOLimelight(camera0Name, drive::getRotation),
            new VisionIOLimelight(camera1Name, drive::getRotation));*/
        // intake = new Intake(new IntakeIOSpark());
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(new DriveIOSim(), new GyroIO() {});
        angulador = new Angulador(new AnguladorIOSim());
        elevador = new Elevador(new ElevadorIOSim());
        // wrist = new Wrist(new WristIOSim());
        /*vision =
        new Vision(
            drive::addVisionMeasurement,
            new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
            new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose)); */
        // intake = new Intake(new IntakeIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(new DriveIO() {}, new GyroIO() {});
        angulador = new Angulador(new AnguladorIO() {});
        elevador = new Elevador(new ElevadorIO() {});
        // wrist = new Wrist(new WristIO() {});
        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        // intake = new Intake(new IntakeIO() {});
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
    drive.setDefaultCommand(
        DriveCommands.arcadeDrive(
            drive, () -> -controller.getLeftY(), () -> -controller.getRightX()));

    angulador.setDefaultCommand(ArmCommands.controlArm(angulador, () -> controller.getRightY()));

    elevador.setDefaultCommand(
        ElevatorCommands.controlElevator(
            elevador, () -> (controller.getRightTriggerAxis() - controller.getLeftTriggerAxis())));

    /*wrist.setDefaultCommand(
    WristCommands.controlWrist(
        wrist,
        () -> (controller.getLeftTriggerAxis() - controller.getRightTriggerAxis()) * .25)); */

    // controller.y().onTrue(WristCommands.setAngle(wrist, 0));
    // controller.x().onTrue(WristCommands.setAngle(wrist, 90));

    // Configure button to run intake
    controller.leftBumper().onTrue(ArmCommands.setAngle(angulador, 0));

    // Configure button to run intake in reverse
    controller.rightBumper().onTrue(ArmCommands.setAngle(angulador, 90));

    // Configure button to run the pathfinding command
    controller.x().onTrue(command);
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
