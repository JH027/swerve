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

  private SwerveDriveKinematics kinematics;
  private SwerveDriveOdometry m_odometry;

  private Translation2d m_frontLeftLocation; 
  private Translation2d m_frontRightLocation;
  private Translation2d m_backLeftLocation;
  private Translation2d m_backRightLocation; 

  private SwerveModuleState frontLeftState;
  private SwerveModuleState frontRightState;
  private SwerveModuleState backLeftState;
  private SwerveModuleState backRightState;

  public drivetrain() {
    front_left = new swerve(10,11,13);
    front_right = new swerve(20,21,23);
    back_left = new swerve(30,31,33);
    back_right = new swerve(40,41,43);

    m_gyro = new AHRS(SPI.Port.kMXP);
    pose = new Pose2d();
    m_odometry = new SwerveDriveOdometry(kinematics, m_gyro.getRotation2d(), new SwerveModulePosition[] {front_left.position(), front_right.position(),back_left.position(), back_right.position()});

    m_frontLeftLocation = new Translation2d(-SwerveConstants.distance, SwerveConstants.distance);
    m_frontRightLocation = new Translation2d(SwerveConstants.distance, SwerveConstants.distance);
    m_backLeftLocation = new Translation2d(-SwerveConstants.distance, -SwerveConstants.distance);
    m_backRightLocation = new Translation2d(SwerveConstants.distance, -SwerveConstants.distance);

    kinematics = new SwerveDriveKinematics(m_frontLeftLocation,m_frontRightLocation,m_backLeftLocation,m_backRightLocation);

    frontLeftState = front_left.getState();
    frontRightState = front_right.getState();
    backLeftState = back_left.getState();
    backRightState = back_right.getState();

    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetPose,
      () -> kinematics.toChassisSpeeds(frontLeftState, frontRightState, backLeftState, backRightState), 
      this::move2,
      new HolonomicPathFollowerConfig( 
        new PIDConstants(0, 0.0, 0.0), 
        new PIDConstants(0, 0.0, 0.0), 
        4.5, 
        0.3429, 
        new ReplanningConfig() 
      ),
      () -> {
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        return alliance.get() == DriverStation.Alliance.Red;
      }
      return false;
      },
      this // Reference to this subsystem to set requirements
    );
  }

  public void move(double forward, double side, double rotation){
    ChassisSpeeds speeds = new ChassisSpeeds(forward, side, rotation);
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    front_left.setDesiredStates(states[0]);
    front_right.setDesiredStates(states[1]);
    back_left.setDesiredStates(states[2]);
    back_right.setDesiredStates(states[3]);
  }
  public void move2(ChassisSpeeds speeds){
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    front_left.setDesiredStates(states[0]);
    front_right.setDesiredStates(states[1]);
    back_left.setDesiredStates(states[2]);
    back_right.setDesiredStates(states[3]);
  }

  public Pose2d getPose(){
    return new Pose2d(new Translation2d(m_odometry.getPoseMeters().getX(),m_odometry.getPoseMeters().getY()), m_gyro.getRotation2d());
  }
  public void resetPose(Pose2d currentPose){
    m_odometry.resetPosition(m_gyro.getRotation2d(),  new SwerveModulePosition[] {
      front_left.position(), front_right.position(),back_left.position(), back_right.position()},
       currentPose);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  // m_odometry.update(m_gyro.getRotation2d(), new SwerveModulePosition[] {front_left.position(), front_right.position(),back_left.position(), back_right.position()});
  }
}
