// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  private swerve front_left;
  private swerve front_right;
  private swerve back_left;
  private swerve back_right;

  private AHRS m_gyro;
  private Pose2d pose;


  private SwerveDriveKinematics swerve1;
  private SwerveDriveOdometry m_odometry;

  private Translation2d m_frontLeftLocation; 
  private Translation2d m_frontRightLocation;
  private Translation2d m_backLeftLocation;
  private Translation2d m_backRightLocation; 

  public drivetrain() {
    front_left = new swerve(1,2);
    front_right = new swerve(3,4);
    back_left = new swerve(5,6);
    back_right = new swerve(7,8);

    m_gyro = new AHRS();
    pose = new Pose2d();
    m_odometry = new SwerveDriveOdometry(swerve1, m_gyro.getRotation2d(), new SwerveModulePosition[] {front_left.position(), front_right.position(),back_left.position(), back_right.position()});

    m_frontLeftLocation = new Translation2d(SwerveConstants.distance, SwerveConstants.distance);
    m_frontRightLocation = new Translation2d(SwerveConstants.distance, -SwerveConstants.distance);
    m_backLeftLocation = new Translation2d(-SwerveConstants.distance, SwerveConstants.distance);
    m_backRightLocation = new Translation2d(-SwerveConstants.distance, -SwerveConstants.distance);

    swerve1 = new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);

  }

  public void move(double forward, double side, double rotation){
    ChassisSpeeds speeds = new ChassisSpeeds(forward, side, rotation);
    SwerveModuleState[] states = swerve1.toSwerveModuleStates(speeds);
    front_left.setDesiredStates(states[0]);
    front_right.setDesiredStates(states[1]);
    back_left.setDesiredStates(states[2]);
    back_right.setDesiredStates(states[3]);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {front_left.position(), front_right.position(),back_left.position(), back_right.position()});
  }
}
