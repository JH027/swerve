package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX leftMotor;
    private final TalonFX rightMotor;

    public Intake() {
        leftMotor = new TalonFX(50);
        rightMotor = new TalonFX(51);
    }

    public void intake(double speed) {
        leftMotor.set(speed);
        rightMotor.set(speed);
    }

    public void outtake(double speed) {
        leftMotor.set(-speed);
        rightMotor.set(speed);
    }

    public void stop() {
        leftMotor.set(0);
        rightMotor.set(0);
    }
}
