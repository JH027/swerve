// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain;

public class driving extends Command {
  /** Creates a new driving. */
  private final drivetrain m_drivetrain;
  private final CommandXboxController m_driverController; 
  public driving(drivetrain m_drivetrain, CommandXboxController m_driverController) {
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
    double leftY = m_driverController.getLeftY()<0.05 ? 0:m_driverController.getLeftY();
    double leftX = m_driverController.getLeftX()<0.05 ? 0:m_driverController.getLeftX();
    double rightX = m_driverController.getRightX()<0.05 ? 0:m_driverController.getRightX();
    m_drivetrain.move(leftY, leftX, rightX); 
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
