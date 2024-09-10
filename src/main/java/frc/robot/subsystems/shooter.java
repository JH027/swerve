// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.Timer;

public class Shooter extends SubsystemBase {
  /** Creates a new shooter. */
  private final CANSparkMax motor1;
  private final CANSparkMax motor2;
  private final Timer time = new Timer();
  
  public Shooter() {
    motor1 = new CANSparkMax(1,MotorType.kBrushless);
    motor2 = new CANSparkMax(2,MotorType.kBrushless);
  }
  public void shoot(double speed){
    motor1.set(speed);
    motor2.set(-speed);
  }
  public void timed(double stop,double speed){
    time.start();
    while(time.get()<stop){
      shoot(speed);
    }
    stop();
  }
  public void stop(){
    motor1.set(0);
    motor2.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}