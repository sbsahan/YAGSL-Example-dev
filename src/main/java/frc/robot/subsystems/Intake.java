package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//TODO: add commands to turn intake on, off, reverse and for duration (if needed)

public class Intake extends SubsystemBase {
    TalonFX m_grab;
    TalonFX m_raise;

    public Intake(int grabID, int raiseID) {
        m_grab = new TalonFX(grabID);
        m_raise = new TalonFX(raiseID);
    }

    public void grab(){
        
    }

    public void raise(){
        
    }

    public void reverse(){
        
    }
}
