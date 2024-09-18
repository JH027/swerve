// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.JoystickDrive;
import frc.robot.commands.loadNote;
import frc.robot.commands.passNote;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.subsystems.Shooter;
import frc.robot.commands.shoot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import 
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final Drivetrain m_drivetrain = new Drivetrain();
  private final Intake m_intake = new Intake();
  private final Indexer indexer = new Indexer();
  private final Shooter m_shooter = new Shooter();
  private final SendableChooser<Command> autoChooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // NamedCommands.registerCommand("shoot", new shoot(shooter));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    NamedCommands.registerCommand("intake", new loadNote(m_intake, m_driverController));
    NamedCommands.registerCommand("index", new passNote(indexer, m_driverController));
    NamedCommands.registerCommand("shoot", new shoot(m_shooter));
    // Configure the trigger bindings
    m_drivetrain.setDefaultCommand(new JoystickDrive(m_drivetrain,m_driverController));
    configureBindings();
  }

  /**
  //  * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureDrivetrainBindings(){
  }
  private void configureIndexerBindings(){
    m_driverController.rightBumper().onTrue(indexer.runOnce(() -> indexer.run(0.3)));
    m_driverController.leftBumper().onTrue(indexer.runOnce(() -> indexer.stop()));
  }
  private void configureIntakeBindings(){
    // m_intake.setDefaultCommand(new loadNote(m_intake,m_driverController));
    m_driverController.b().onTrue(m_intake.runOnce(() -> m_intake.intake(0.3)));
    m_driverController.x().onTrue(m_intake.runOnce(() -> m_intake.intake(-0.3)));
    m_driverController.a().onTrue(m_intake.runOnce(() -> m_intake.stop()));

  }
  private void configureBindings() {
    
    // m_driverController.b().onTrue(Commands.sequence(
    //   m_intake.runOnce(() -> m_intake.intake(0.5)),
    //   indexer.runOnce(() -> indexer.run(0.3))
    // ));
    m_driverController.a().onTrue(m_intake.runOnce(() -> m_intake.outtake(0.1)));
    m_driverController.x().onTrue(m_intake.runOnce(() -> m_intake.stop()));
    m_driverController.y().onTrue(indexer.runOnce(() -> indexer.stop()));
    // m_driverController.leftBumper().onTrue(m)

    // m_driverController.rightBumper().onTrue();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
 
}
