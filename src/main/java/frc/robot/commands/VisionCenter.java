package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class VisionCenter extends Command {
    SwerveSubsystem v_swerveSubsystem;
    double tx, ta;
    boolean a = true;
    
    public VisionCenter(SwerveSubsystem system){
        v_swerveSubsystem = system;
        addRequirements(v_swerveSubsystem, system);
        
    }

    @Override
    public void execute(){
        
        tx = LimelightHelpers.getTX("");
        ta = LimelightHelpers.getTA("");
        double xCorrection = tx * VisionConstants.kP;
        double yCorrection = ta * VisionConstants.kP;
        v_swerveSubsystem.drive(new ChassisSpeeds(xCorrection, yCorrection, 0));
        SmartDashboard.putNumber("correctionX", xCorrection);
        SmartDashboard.putNumber("correctionY", yCorrection);
        SmartDashboard.putNumber("valueX", LimelightHelpers.getTX(""));
        SmartDashboard.putNumber("valueY", ta);
        System.out.println("centering");
        //if(tx == 0 && ta == 10) {
            //a = false;
        //}
        
    }

    @Override
    public boolean isFinished(){
        return !a;
    }
}
