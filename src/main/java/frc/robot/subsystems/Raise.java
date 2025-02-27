package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

//TODO: add commands to turn intake on, off, reverse and for duration (if needed)

public class Raise extends SubsystemBase {
    TalonFX m_raise;

    MotionMagicVoltage m_request = new MotionMagicVoltage(0);
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    public Raise(int raiseID) {
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.1; // Add 0.25 V output to overcome static friction
        slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
        slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
        slot0Configs.kP = 1; // A position error of 2.5 rotations results in 12 V output
        slot0Configs.kI = 0; // no output for integrated error
        slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
        motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

        m_raise = new TalonFX(raiseID);

        m_raise.getConfigurator().apply(talonFXConfigs);
    }

    public Command setArmPosition(double position){
        return Commands.runOnce(() -> m_raise.setControl(m_request.withPosition(position)));
    }

    public Command raiseArm(){
        return setArmPosition(IntakeConstants.INTAKE_ARM_POSITION);
    }

    public Command resetArm(){
        return setArmPosition(IntakeConstants.INTAKE_RESET_POSITION);
    }
}
