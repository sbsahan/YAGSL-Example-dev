package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorToLevel extends Command {
    
    private Elevator m_elevatorSubsystem;
    private int e_level;
    
    public ElevatorToLevel(Elevator system, int level) {
        m_elevatorSubsystem = system;
        e_level = level;
        addRequirements(system);
    }
    
    @Override
    public void initialize(){
        switch (e_level) {
            case 1:
                m_elevatorSubsystem.L1().schedule();
                break;
            case 2:
                m_elevatorSubsystem.L2().schedule();
                break;
            case 3:
                m_elevatorSubsystem.L3().schedule();
                break;
            case 0:
                m_elevatorSubsystem.L0().schedule();
                break;
        }
    }

    @Override
    public void execute(){}

    @Override
    public boolean isFinished() {
      return true;
    }
}
