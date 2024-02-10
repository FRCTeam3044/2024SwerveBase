    package frc.robot.commands;

    import com.revrobotics.SparkPIDController;

    import edu.wpi.first.wpilibj.XboxController;
    import edu.wpi.first.wpilibj2.command.Command;
    import frc.robot.subsystems.ElevatorSubsystem;

    public class ElevatorSetAngleForAmpCommand extends Command {
        private final ElevatorSubsystem m_elevator;
        public double ampAngle;

        public ElevatorSetAngleForAmpCommand(ElevatorSubsystem elevator, XboxController controller) {
            m_elevator = elevator;
            addRequirements(m_elevator);
        }
    
        @Override
        public boolean isFinished() {
            return false;
        }

        @Override
        public void initialize() {
            ampAngle = m_elevator.ampAngle;
        }

        @Override
        public void execute() {
            m_elevator.pidHandler(ampAngle);
        }
    }
