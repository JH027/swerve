// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  /** Creates a new Indexer. */
  private final TalonFX motor;
  private final DigitalInput beam;
  public Indexer() {
    motor = new TalonFX(52);
    beam = new DigitalInput(1);
  }
  public void run(double speed){
    motor.set(speed);
  }
  public void beamRun(double speed){
    while(!beam.get()){
      run(speed);
    }
    run(0);
  }
  public Boolean stuck(){
    return beam.get();
  }
  public void stop(){
    run(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}