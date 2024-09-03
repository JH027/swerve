// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new intake. */
  private final TalonFX right;
  private final TalonFX left;
  private final Timer time;

  public Intake() {
    right = new TalonFX(50);
    left = new TalonFX(51);
    time = new Timer();
  }
  public void in(double speed){
    right.set(-speed);
    left.set(speed);
  }
  public void timed(double stop,double speed){
    time.start();
    while(time.get()<stop){
      in(speed);
    }
    stop();
  }
  public void stop(){
    right.set(0);
    left.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
