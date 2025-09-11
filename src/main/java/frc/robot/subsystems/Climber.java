package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

    private final SparkMax climbMotor;

    private SparkClosedLoopController closedLoopController;
    private SparkMaxConfig motorConfig;
    private RelativeEncoder encoder;

    /**
     * This subsytem that controls the climber.
     */
    public Climber() {

    // Set up the climb motor as a brushless motor
    climbMotor = new SparkMax(ClimberConstants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    closedLoopController = climbMotor.getClosedLoopController();
    encoder = climbMotor.getEncoder();

    // // Set can timeout. Because this project only sets parameters once on
    // // construction, the timeout can be long without blocking robot operation. Code
    // // which sets or gets parameters during operation may need a shorter timeout.
    // climbMotor.setCANTimeout(250);

    // // Create and apply configuration for climb motor. Voltage compensation helps
    // // the climb behave the same as the battery
    // // voltage dips. The current limit helps prevent breaker trips or burning out
    // // the motor in the event the climb stalls.
    // SparkMaxConfig climbConfig = new SparkMaxConfig();
    // climbConfig.voltageCompensation(ClimberConstants.CLIMBER_MOTOR_VOLTAGE_COMP);
    // climbConfig.smartCurrentLimit(ClimberConstants.CLIMBER_MOTOR_CURRENT_LIMIT);
    // climbConfig.idleMode(IdleMode.kBrake);
    // climbMotor.configure(climbConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motorConfig = new SparkMaxConfig();
        motorConfig.encoder.positionConversionFactor(1).velocityConversionFactor(1);

        motorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(1)
        .i(0)
        .d(0)
        .outputRange(-1, 1)
        .p(1, ClosedLoopSlot.kSlot1)
        .i(0, ClosedLoopSlot.kSlot1)
        .d(0, ClosedLoopSlot.kSlot1)
        .velocityFF(1/ 5767, ClosedLoopSlot.kSlot1)
        .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        climbMotor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SmartDashboard.setDefaultNumber("Target Position", 0);
        SmartDashboard.setDefaultNumber("Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Control Mode", false);
        SmartDashboard.setDefaultBoolean("Reset Encoder", false);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbMotor Rotation", climbMotor.getEncoder().getPosition());
    }

    public Command climbOut(double move) {
        return this.runOnce(() -> {closedLoopController.setReference(move, ControlType.kPosition, ClosedLoopSlot.kSlot1);});
    }

    /**
     * Use to run the climber, can be set to run from 100% to -100%.
     * Keep in mind that the direction changes based on which way the winch is wound.
     * 
     * @param speed motor speed from -1.0 to 1, with 0 stopping it
     */
    public void runClimber(double speed){
        climbMotor.set(speed);
    }

    public Command ClimbStop() {
        return this.runOnce(() -> { runClimber(0);});
    }

    public Command motorGoOutAtEndGame() {
        return this.runOnce(() -> { runClimber(0);});
    }

}