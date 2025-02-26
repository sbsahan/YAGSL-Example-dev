package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionDistance extends Command {
    SwerveSubsystem v_swerveSubsystem;
    
    VisionDistance(SwerveSubsystem system){
        v_swerveSubsystem = system;
        addRequirements(v_swerveSubsystem);
    }

    @Override
    public void execute(){
        //v_swerveSubsystem.driveToPose();
    }
}
