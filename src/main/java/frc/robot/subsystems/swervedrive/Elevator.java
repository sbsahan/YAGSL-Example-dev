package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: add commands to set height for scoring [1, 2, 3]

public class Elevator extends SubsystemBase{
    TalonFX elevatorMotor; //changed based on motor type

    public Elevator(int motorID){
        elevatorMotor = new TalonFX(motorID);
    }
}
