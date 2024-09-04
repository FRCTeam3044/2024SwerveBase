package frc.robot.statemachine;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.utils.AutoTargetUtils;

public class StateCommands {
    public static Command goToSource() {
        return RobotContainer.m_robotDrive.goToAndTrackPoint(AutoTargetUtils.getSource(),
                AutoTargetUtils.getSourceTrackTarget(), false, false);
    }
}
