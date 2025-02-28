package frc.robot.commands.elevator;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorManual extends Command {
    
    private Elevator m_elevatorSubsystem;
    private DoubleSupplier e_axis;
    
    public ElevatorManual(Elevator system, DoubleSupplier axis){
        m_elevatorSubsystem = system;
        e_axis = axis;
        addRequirements(system);
    }
    @Override
    public void execute(){
        double pow = e_axis.getAsDouble()*-1;
        m_elevatorSubsystem.setManual(pow);
        System.out.println("manual command default working!");
    }
}
