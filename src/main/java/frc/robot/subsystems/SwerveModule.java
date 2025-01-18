package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Configs;

public class SwerveModule {

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final SparkClosedLoopController drivingClosedLoopController;
    private final SparkClosedLoopController turningClosedLoopController;

    private double chassisAngularOffset = 0;

    final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    private final SlewRateLimiter filter;


    public SwerveModule(int driveMotorId, int turningMotorId,
            int absoluteEncoderId, double absoluteEncoderOffset, 
            boolean absoluteEncoderReversed, double chassisAngularOffset) {

        driveMotor = new SparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorId, MotorType.kBrushless);
        
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        drivingClosedLoopController = driveMotor.getClosedLoopController();
        turningClosedLoopController = turningMotor.getClosedLoopController();

        driveMotor.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turningMotor.configure(Configs.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        filter = new SlewRateLimiter(1.45);

        chassisAngularOffset = chassisAngularOffset;

        resetEncoders();
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition() - chassisAngularOffset));
    }

    
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), new Rotation2d(getAbsoluteEncoderRad() - chassisAngularOffset));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        SwerveModuleState correctedDesiredState = new SwerveModuleState();
        correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
        correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

        correctedDesiredState.optimize(new Rotation2d(turningEncoder.getPosition()));

        drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
        turningClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

        desiredState = desiredState;

    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public double getDrivePosition() {  
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        angle = angle * (absoluteEncoderReversed ? -1.0 : 1.0);
        return MathUtil.inputModulus(angle, 0, 2.0 * Math.PI);
    }

    // public void setDesiredStateAuto(SwerveModuleState state) {
    //     int index = 0;
    //     if (Math.abs(state.speedMetersPerSecond) < 0.001) {
    //         stop();
    //         return;
    //     }
    //     state = SwerveModuleState.optimize(state, getState().angle);
    //     driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    //     turningMotor.set(state.angle.getRadians());

    //     if (index%10 == 0) {
    //         turningEncoder.setPosition(getAbsoluteEncoderRad());
    //     }
    //         index = index + 1;

    //     SmartDashboard.putString("SwerveAuto[" + absoluteEncoder.getChannel() + "] state", state.toString());
    // }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
    public AnalogInput getEncoder() {
        return absoluteEncoder;
    }
}
