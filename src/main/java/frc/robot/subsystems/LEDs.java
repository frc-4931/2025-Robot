package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDs extends SubsystemBase {
    
    private double color = 0.89;
    private Spark LEDController = new Spark(0);

    @Override
    public void periodic() {
        LEDController.set(color);
    }

    public void setColor(double newcolor) {
        LEDController.set(newcolor);

    }
}