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
    double shooterAngle = 0;

    boolean isShooterRunning = false;

    public void shooterFiring(double motorRPM) {

    }

    public void shooterAngleAdjustment(double shooterAngle) {

    }
}
