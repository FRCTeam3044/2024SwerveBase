package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

public class ShooterSubsystem {
    /*
     * Defines the motors that shoot the notes
     */
    CANSparkMax rightMotorFront = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax rightMotorBottom = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax leftMotorFront = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax leftMotorBottom = new CANSparkMax(0, MotorType.kBrushless);

    /*
     * Defines the motors that change the angle of the shooter
     */
    CANSparkMax shooterAngleMotorOne = new CANSparkMax(0, MotorType.kBrushless);
    CANSparkMax shooterAngleMotorTwo = new CANSparkMax(0, MotorType.kBrushless);

    /*
     * Encoders for Shooter wheels
     */
    RelativeEncoder rightFrontMotorEncoder = rightMotorFront.getEncoder();
    RelativeEncoder rightBottomkMotorEncoder = rightMotorBottom.getEncoder();
    RelativeEncoder leftFrontMotorEncoder = leftMotorFront.getEncoder();
    RelativeEncoder leftBottomMotorEncoder = leftMotorBottom.getEncoder();

    /*
     * Encoders for the angle control
     */
    AbsoluteEncoder shooterAngleMotorOneEncoder = shooterAngleMotorOne.getAbsoluteEncoder(Type.kDutyCycle);
    AbsoluteEncoder shooterAngleMotorTwoEncoder = shooterAngleMotorTwo.getAbsoluteEncoder(Type.kDutyCycle);

    /*
     * Change this to change the power of the motors
     */
    double motorRPM = 0;

    /*
     * Change this to change the angles of the shooters
     */
    double desiredShooterAngle = 0;
    /*
     * Base rotations per minute
     */
    double targetRPM = 0;

    /*
     * Sets the rpm
     */
    public void setTargetRPM(double motorRPM) {

    }

<<<<<<< HEAD
    }
=======
    /*
     * Sets the angle of the shooter for shooting
     */
    public void setShooterAngle(double shooterAngle) {
>>>>>>> e4d0ef73e10f0ebfe2797328b6c47f7656e4c523

    }

<<<<<<< HEAD
=======
    /*
     * Updates the shooter's information
     */
    public void updatePeriodic() {

>>>>>>> e4d0ef73e10f0ebfe2797328b6c47f7656e4c523
    }
}
