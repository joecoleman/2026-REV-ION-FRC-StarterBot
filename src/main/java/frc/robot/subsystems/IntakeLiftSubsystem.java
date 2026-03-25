package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeLiftSubsystem extends SubsystemBase {
    private final SparkMax liftMotor = new SparkMax(7, MotorType.kBrushless);

    public IntakeLiftSubsystem() {
        // Optionally configure motor settings here
    }

    /**
     * Set the lift motor power (-1.0 to 1.0)
     */
    public void setLiftPower(double power) {
        liftMotor.set(power);
    }

    /**
     * Stop the lift motor
     */
    public void stopLift() {
        liftMotor.set(0.0);
    }
}
