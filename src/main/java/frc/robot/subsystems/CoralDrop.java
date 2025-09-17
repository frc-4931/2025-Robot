package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.Command;
import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralDrop extends SubsystemBase{

    private SparkClosedLoopController closedLoopController;
    private boolean isRunning;

    public CoralDrop(){

    }

    public Command toggleRun(){
        return this.runOnce(() -> {
            isRunning = !isRunning;
            setSpeed();;
        });
        //(() -> {isRunning = !(isRunning); setSpeed();});
    }

    public void setSpeed(){
        if(isRunning) {
       closedLoopController.setReference(-10, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
       
        }
        else{
            closedLoopController.setReference(0, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
        }
    }

    public void stop(){
        closedLoopController.setReference(0, ControlType.kVoltage, ClosedLoopSlot.kSlot1);
    }
}