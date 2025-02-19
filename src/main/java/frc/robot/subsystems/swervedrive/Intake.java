package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: add commands to turn intake on, off, reverse and for duration (if needed)

public class Intake extends SubsystemBase {
    TalonFX intakeMotor;

    public Intake(int motorID) {
        intakeMotor = new TalonFX(motorID);
    }
}
