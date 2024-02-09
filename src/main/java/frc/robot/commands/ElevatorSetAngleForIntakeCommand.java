    package frc.robot.commands;

    import com.revrobotics.SparkPIDController;

    import edu.wpi.first.wpilibj.XboxController;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.ElevatorSubsystem;

    public class ElevatorSetAngleForIntakeCommand extends Command {
        private final ElevatorSubsystem m_elevator;
        public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
        public SparkPIDController pidController;


        public ElevatorSetAngleForIntakeCommand(ElevatorSubsystem elevator, XboxController controller, SparkPIDController pidController) {
            m_elevator = elevator;
            addRequirements(m_elevator);
        }
        
        // @Override
        // public void initialize() {
        //     double maxVel = 0;
        //     double maxAccel = 0;

        //     // PID coefficients
        //     kP = 0.1;
        //     kI = 0;
        //     kD = 1;
        //     kIz = 0;
        //     kFF = 0;
        //     kMaxOutput = 0.1;
        //     kMinOutput = -0.1;

        //     pidController = m_elevator.elevatorMotorOne.getPIDController();

        //     m_pidController.setP(kP);
        //     m_pidController.setI(kI);
        //     m_pidController.setD(kD);
        //     m_pidController.setIZone(kIz);
        //     m_pidController.setFF(kFF);
        //     m_pidController.setOutputRange(kMinOutput, kMaxOutput);
        //     m_pidController.setSmartMotionMaxVelocity(maxVel, 0);
        //     m_pidController.setSmartMotionMaxAccel(maxAccel, 0);
        // }

        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void execute() {
            m_elevator.pidHandler(m_elevator.intakeAngle);
        }
    }
