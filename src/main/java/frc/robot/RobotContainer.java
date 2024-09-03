// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.driving;
import frc.robot.commands.loadNote;
import frc.robot.commands.passNote;
import frc.robot.commands.shoot;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drivetrain;
import frc.robot.subsystems.shooter;
import frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.*;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
   public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("swerve");
    return AutoBuilder.followPath(path);
    // An example command will be run in autonomous
    //  return Autos.exampleAuto(m_exampleSubsystem);
  }
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final drivetrain m_drivetrain = new drivetrain();
  private final shooter m_shooter = new shooter();
  private final Indexer m_indexer = new Indexer();
  private final Intake m_intake = new Intake();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final XboxController m_driverController = new XboxController(OperatorConstants.kDriverControllerPort);
  private final SendableChooser<Command> autoChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("shoot", new shoot(m_shooter));
    NamedCommands.registerCommand("intake", new loadNote(m_intake, m_driverController));
    NamedCommands.registerCommand("indexer", new passNote(m_indexer, m_driverController));
    // Configure the trigger bindings
    configureDrivetrainBindings();
    configureIndexerBindings();
    configureIntakeBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);  
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
    m_drivetrain.setDefaultCommand(new driving(m_drivetrain,m_driverController));
  }
  private void configureIndexerBindings(){
    m_indexer.setDefaultCommand(new passNote(m_indexer, m_driverController));
  }
  private void configureIntakeBindings(){

  }
  private void configureBindings() {
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
 
}
