package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;

public class Vision extends SubsystemBase {
    String v_name, v_side;
    int v_pipeline;
    double tx, ta, X_Correction, Y_Correction;

    
    public Vision(String name, String side){
        v_name = name;
        v_side = side;
        // tx = LimelightHelpers.getTX(v_name);
        // ty = LimelightHelpers.getTY(v_name);
        // ta = LimelightHelpers.getTA(v_name);
    }

    public void updateValues(){
        tx = LimelightHelpers.getTX(v_name);
        ta = LimelightHelpers.getTA(v_name);
    }

    public void publishValues(){
        SmartDashboard.putNumber(v_name + "tx", tx);
        SmartDashboard.putNumber(v_name + "ta", ta);
    }

    public void calculateCorrection(){
        double x_Correction;
        double y_Correction = (VisionConstants.TA_GOAL - ta) * VisionConstants.kP;
        switch (v_side) {
            case "left":
                x_Correction = (VisionConstants.TX_GOAL_LEFT - tx) * VisionConstants.kP;
                break;

            case "right":
                x_Correction = (VisionConstants.TX_GOAL_RIGHT - tx) * VisionConstants.kP;
                break;

            default:
                System.out.println("aw hell naw dawg");
                x_Correction = 0;
        }
        X_Correction = x_Correction;
        Y_Correction = y_Correction;
    }

    public double getTX(){
        return tx;
    }

    public double getTA(){
        return ta;
    }

    public double getX_Correction(){
        return X_Correction;
    }

    public double getY_Correction(){
        return Y_Correction;
    }
}
