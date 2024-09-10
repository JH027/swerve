// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AnalogInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class swerve extends SubsystemBase {
  /** Creates a new swerve. */


  SwerveDriveKinematics swerve;

  private final TalonFX drive;
  private final TalonFX turn;

  private final CANcoder turnEncoder;
  
  private final PIDController m_piddrive;
  private final PIDController m_pidturn;

  
  public swerve(int driveId, int turnId, int cancoderId) {
    drive = new TalonFX(driveId);
    turn = new TalonFX(turnId);

    turnEncoder = new CANcoder(cancoderId,"CANivore");

    m_pidturn = new PIDController(0, 0, 0);
    m_piddrive = new PIDController(0.01, 0,0);
  }

  public void setDesiredStates(SwerveModuleState state){
    
    turn.set(m_pidturn.calculate(turnEncoder.getPosition().getValueAsDouble(),state.angle.getRadians()));
    drive.set(m_piddrive.calculate(drive.getVelocity().getValueAsDouble(),state.speedMetersPerSecond));
    SmartDashboard.putNumber("output", m_pidturn.calculate(turnEncoder.getPosition().getValueAsDouble(),state.angle.getRadians()));
    
    /*
    turn.set(MathUtil.clamp(state.angle.getRadians(),-0.5,0.5));
    drive.set(MathUtil.clamp(state.speedMetersPerSecond,-0.5, 0.5));
    SmartDashboard.putNumber("turn",state.angle.getRadians());
    SmartDashboard.putNumber("drive",state.speedMetersPerSecond);
    */
  }
  public SwerveModuleState getState(){
    return new SwerveModuleState(drive.getVelocity().getValueAsDouble(),new Rotation2d(turnEncoder.getPosition().getValueAsDouble()));
  }
  

  public SwerveModulePosition position(){
    return new SwerveModulePosition(drive.getPosition().getValueAsDouble(),new Rotation2d(turnEncoder.getPosition().getValueAsDouble()));
  }

  public void stop(){
    drive.set(0);
    turn.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
