package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: add commands to set height for scoring [1, 2, 3]

public class Elevator extends SubsystemBase{
    
    TalonFX elevatorMotor1; //changed based on motor type
    TalonFX elevatorMotor2;

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    public Elevator(int motorID1, int motorID2){
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 1; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.5; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.05; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 20; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)
        
        elevatorMotor1 = new TalonFX(motorID1);
        elevatorMotor2 = new TalonFX(motorID2);

        elevatorMotor1.getConfigurator().apply(talonFXConfigs);
        elevatorMotor2.setControl(new Follower(motorID1, true));  
    
    }
    
    public void elevatorPosOne(){
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        elevatorMotor1.setControl(m_request.withPosition(10));
    }

    public void elevatorPosTwo(){
        final MotionMagicVoltage m_request = new MotionMagicVoltage(0);
        elevatorMotor1.setControl(m_request.withPosition(20));
    }

    public void elevatorReset(){
        elevatorMotor1.setPosition(0);
        System.out.println("worked!");
    }

}

