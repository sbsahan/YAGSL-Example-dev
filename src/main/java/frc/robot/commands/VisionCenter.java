package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCenter extends Command {
    SwerveSubsystem v_swerveSubsystem;
    Vision v_ll;
    
    public VisionCenter(SwerveSubsystem system, Vision ll){
        v_swerveSubsystem = system;
        v_ll = ll;
        addRequirements(system, ll);
    }

    @Override
    public void execute(){
        v_ll.updateValues();
        v_swerveSubsystem.drive(new ChassisSpeeds(v_ll.getX_Correction(), v_ll.getY_Correction(), 0));
        v_ll.publishValues();
        System.out.println("centering");
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}
