package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Raise;


public class RaiseCoral extends Command {
    private Raise i_intakeSubsystem;
    
    RaiseCoral(Raise system){
        i_intakeSubsystem = system;
        addRequirements(system);
    }

    @Override
    public void initialize(){
        i_intakeSubsystem.raiseArm().schedule();
    }

    @Override
    public boolean isFinished(){
        return true;
    }

}