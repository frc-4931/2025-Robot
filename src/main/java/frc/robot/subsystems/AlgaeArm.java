package frc.robot.subsystems;

import java.io.ObjectInputFilter.Config;

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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class AlgaeArm extends SubsystemBase{
    private SparkMax wheelMotor;
    private SparkMax moveMotor;
    private SparkMaxConfig motorConfig;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private boolean isRunning;

    public AlgaeArm(boolean Phase2){
        if (Phase2 = true) {
            wheelMotor = new SparkMax(11, MotorType.kBrushless);
            moveMotor = new SparkMax(12, MotorType.kBrushless);

            SparkMaxConfig wheelMotorConfig = new SparkMaxConfig();
            SparkMaxConfig moveMotorConfig = new SparkMaxConfig();

            wheelMotorConfig
                .smartCurrentLimit(50)
                .idleMode(IdleMode.kBrake)
                .inverted(true);
            
            moveMotorConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake)
            .inverted(true);

            wheelMotor.configure(wheelMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            moveMotor.configure(moveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        }
            
    else {
            
        }
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