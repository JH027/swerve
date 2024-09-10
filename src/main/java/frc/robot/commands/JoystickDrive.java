// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Drivetrain;

public class JoystickDrive extends Command {
  /** Creates a new driving. */
  private final Drivetrain m_drivetrain;
  private final CommandXboxController m_driverController; 

  public JoystickDrive(Drivetrain m_drivetrain, CommandXboxController m_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_drivetrain = m_drivetrain;
    this.m_driverController = m_driverController;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // m_drivetrain.move(Math.abs(m_driverController.getLeftY())<0.1 ? 0:m_driverController.getLeftY(), m_driverController.getLeftX()<0.1 ? 0:m_driverController.getLeftX(), m_driverController.getRightX()<0.1 ? 0:m_driverController.getRightX());
    double xSpeed = Math.abs(m_driverController.getLeftY())<0.1 ? 0:m_driverController.getLeftY();
    double ySpeed = Math.abs(m_driverController.getLeftX())<0.1 ? 0:m_driverController.getLeftX();
    double rot = Math.abs(m_driverController.getRightX())<0.1 ? 0:-m_driverController.getRightX();

    m_drivetrain.drive(xSpeed, ySpeed, rot);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("rot", rot);
    SmartDashboard.putNumber("ySpeed", ySpeed);

    Logger.recordOutput("xSpeed", xSpeed);
    Logger.recordOutput("rot", rot);
    Logger.recordOutput("ySpeed", ySpeed);

    // m_drivetrain.move(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
