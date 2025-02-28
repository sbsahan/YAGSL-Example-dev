package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
    TalonFX m_grab;

    public Grabber(int grabID){
        m_grab = new TalonFX(grabID);
    }
    
    
}
