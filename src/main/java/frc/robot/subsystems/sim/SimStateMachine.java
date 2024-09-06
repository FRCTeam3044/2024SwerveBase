package frc.robot.subsystems.sim;

import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NoteDetection;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateMachine;
import frc.robot.subsystems.TransitSubsystem;

public class SimStateMachine extends StateMachine {
    /**
     * Creates a new StateMachine
     * 
     * @param shooterSubsystem  The shooter subsystem
     * @param elevatorSubsystem The elevator subsystem
     * @param transitSubsystem  The transit subsystem
     * @param intakeSubsystem   The intake subsystem
     * @param noteDetection     The note detection subsystem
     * @param driveSubsystem    The drive subsystem
     */
    public SimStateMachine(ShooterSubsystem shooterSubsystem, ElevatorSubsystem elevatorSubsystem,
            TransitSubsystem transitSubsystem, IntakeSubsystem intakeSubsystem,
            NoteDetection noteDetection, DriveSubsystem driveSubsystem) {
        super(shooterSubsystem, elevatorSubsystem, transitSubsystem, intakeSubsystem, noteDetection, driveSubsystem);
    }

    //@Override
    protected boolean noteIn() {
        return RobotContainer.m_driverController.getHID().getBButton();
    }

    @Override
    protected boolean shooterAtSpeed() {
        return RobotContainer.m_driverController.getHID().getXButton();
    }

    @Override
    protected boolean shooterAtAngle() {
        return RobotContainer.m_driverController.getHID().getYButton();
    }
}
