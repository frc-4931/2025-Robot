package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CoralDrop extends SubsystemBase{
    private SparkMax coralMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private boolean isRunning;

    public CoralDrop(){
    //     if (Phase1 = true) {
    //     coralMotor = new SparkMax(9, MotorType.kBrushless);
    //     closedLoopController = coralMotor.getClosedLoopController();
    //     encoder = coralMotor.getEncoder();

    //     motorConfig = new SparkMaxConfig();

    //     motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

    //     motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    //     .p(.1)
    //     .i(0)
    //     .d(0)
    //     .outputRange(-1, 1)
    //     .p(.00001, ClosedLoopSlot.kSlot1)
    //     .i(0, ClosedLoopSlot.kSlot1)
    //     .d(0, ClosedLoopSlot.kSlot1)
    //     .velocityFF(1/ 5767, ClosedLoopSlot.kSlot1)
    //     .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

    //     coralMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    //     SmartDashboard.setDefaultNumber("Target Position", 0);
    //     SmartDashboard.setDefaultNumber("Target Velocity", 0);
    //     SmartDashboard.setDefaultBoolean("Control Mode", false);
    //     SmartDashboard.setDefaultBoolean("Reset Encoder", false);

    //     isRunning = false;
    //     }
            
    // else {
            
    //     }
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