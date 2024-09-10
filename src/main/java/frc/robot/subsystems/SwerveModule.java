// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.CancelablePrintJob;

import com.ctre.phoenix6.configs.MagnetSensorConfigs;
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

public class SwerveModule extends SubsystemBase {
  /** Creates a new swerve. */

  private SwerveDriveKinematics swerve;

  private final TalonFX drive;
  private final TalonFX turn;

  private final CANcoder turnEncoder;
  
  private final PIDController m_piddrive;
  private final PIDController m_pidturn;

  private final MagnetSensorConfigs sensorConfigs = new MagnetSensorConfigs();

  public SwerveModule(int driveId, int turnId, int cancoderId, double magnetOffset) {
    drive = new TalonFX(driveId);
    turn = new TalonFX(turnId);

    turnEncoder = new CANcoder(cancoderId,"CANivore");

    m_pidturn = new PIDController(0.25, 0, 0);
    m_piddrive = new PIDController(0.2,0,0);

    m_pidturn.enableContinuousInput(0, 2 * Math.PI);

    // turn.setInverted(true);
    sensorConfigs.withMagnetOffset(magnetOffset);
  }

  public void setDesiredStates(SwerveModuleState desiredState) {
    double m_moduleAngleRadians = turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI;
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(m_moduleAngleRadians));
    turn.set(MathUtil.clamp(m_pidturn.calculate(m_moduleAngleRadians, state.angle.getRadians()), -0.5, 0.5));
    drive.set(MathUtil.clamp(m_piddrive.calculate(drive.getVelocity().getValueAsDouble()*SwerveConstants.driveConversionFactor, state.speedMetersPerSecond), -0.5, 0.5));

    SmartDashboard.putNumber("turn output", MathUtil.clamp(m_pidturn.calculate(turnEncoder.getPosition().getValueAsDouble(), state.angle.getRadians()), -0.5, 0.5));
    SmartDashboard.putNumber("drive setpoint", state.speedMetersPerSecond);
    SmartDashboard.putNumber("drive encoder velocity", drive.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("turn setpoint", state.angle.getRadians());
    SmartDashboard.putNumber("turn encoder position", m_moduleAngleRadians);
    SmartDashboard.putNumber("drive output", MathUtil.clamp(m_piddrive.calculate(drive.getVelocity().getValueAsDouble(), state.speedMetersPerSecond), -0.5, 0.5));
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(drive.getVelocity().getValueAsDouble(),new Rotation2d(turnEncoder.getPosition().getValueAsDouble()));
  }

  public void stop(){
    drive.set(0);
    turn.set(0);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(drive.getVelocity().getValueAsDouble(), new Rotation2d(turnEncoder.getPosition().getValueAsDouble()));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
