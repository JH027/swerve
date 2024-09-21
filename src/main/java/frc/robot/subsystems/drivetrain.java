// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

import frc.robot.subsystems.SwerveModule;

public class Drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  private SwerveModule front_left;
  private SwerveModule front_right;
  private SwerveModule back_left;
  private SwerveModule back_right;

  private AHRS m_gyro;

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry m_odometry;

  private Translation2d m_frontLeftLocation; 
  private Translation2d m_frontRightLocation;
  private Translation2d m_backLeftLocation;
  private Translation2d m_backRightLocation; 

  public Drivetrain() {
    front_left = new SwerveModule(10, 11, 13, 0.107666015625);
    front_right = new SwerveModule(20, 21, 23, 0.329833984375);
    back_left = new SwerveModule(30, 31, 33, -0.31396484375);
    back_right = new SwerveModule(40, 41, 43, 0.428955078125);

    front_left.getDriveMotor().setInverted(true);
    front_right.getDriveMotor().setInverted(true);
    back_left.getDriveMotor().setInverted(false);
    back_right.getDriveMotor().setInverted(false);

    front_left.getTurnMotor().setInverted(false);
    front_right.getTurnMotor().setInverted(false);
    back_left.getTurnMotor().setInverted(false);
    back_right.getTurnMotor().setInverted(false);

    m_gyro = new AHRS(SPI.Port.kMXP);

    m_frontLeftLocation = new Translation2d(SwerveConstants.distance, SwerveConstants.distance);
    m_frontRightLocation = new Translation2d(SwerveConstants.distance, -SwerveConstants.distance);
    m_backLeftLocation = new Translation2d(-SwerveConstants.distance, SwerveConstants.distance);
    m_backRightLocation = new Translation2d(-SwerveConstants.distance, -SwerveConstants.distance);

    kinematics = new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);

    m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {front_left.getPosition(), front_right.getPosition(),back_left.getPosition(), back_right.getPosition()});
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose,
      () -> kinematics.toChassisSpeeds(front_left.getState(), front_right.getState(), back_left.getState(), back_right.getState()), 
      this::move,
      new HolonomicPathFollowerConfig( 
        new PIDConstants(5, 0.0, 0.0), 
        new PIDConstants(5, 0.0, 0.0), 
        4.5, 
        SwerveConstants.driveRadius, 
        new ReplanningConfig() 
      ),
      () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this
      );
  }

  public void drive(double xSpeed, double ySpeed, double rotation) {
    var swerveModuleStates = kinematics.toSwerveModuleStates(
      new ChassisSpeeds(xSpeed, ySpeed, rotation)
    );
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 5.5);
    front_left.setDesiredStates(swerveModuleStates[0]);
    front_right.setDesiredStates(swerveModuleStates[1]);
    back_left.setDesiredStates(swerveModuleStates[2]);
    back_right.setDesiredStates(swerveModuleStates[3]);
  }

  public void move(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    front_left.setDesiredStates(states[0]);
    front_right.setDesiredStates(states[1]);
    back_left.setDesiredStates(states[2]);
    back_right.setDesiredStates(states[3]);
  }

  public Pose2d getPose() {
    return new Pose2d(new Translation2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY()), new Rotation2d(m_gyro.getAngle()));
  }
  public void resetPose(Pose2d currentPose){
    m_odometry.resetPosition(m_gyro.getRotation2d(),  new SwerveModulePosition[] {
      front_left.getPosition(), front_right.getPosition(),back_left.getPosition(), back_right.getPosition()},
       currentPose);
  }
  public void resetGyro() {
    m_gyro.reset();
  }

  public SwerveDriveOdometry getOdometry() {
    return m_odometry;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(
      m_gyro.getRotation2d(), 
      new SwerveModulePosition[] {
        front_left.getPosition(), 
        front_right.getPosition(),
        back_left.getPosition(), 
        back_right.getPosition()
      });
  }
}
