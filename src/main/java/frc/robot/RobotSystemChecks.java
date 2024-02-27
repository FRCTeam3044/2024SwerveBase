package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotSystemChecks extends TimedRobot {
    private static RobotSystemChecks instance;

    public void robotInit() {
        instance = this;
        addPeriodic(() -> SmartDashboard.putNumber("MatchTime", DriverStation.getMatchTime()), 1.0);
    }

    public void robotPeriodic() {
        SmartDashboard.putNumber("RIOInputVoltage", RobotController.getInputVoltage());
        SmartDashboard.putNumber("RIOCANUtil", RobotController.getCANStatus().percentBusUtilization * 100);
    }

    public static void addPeriodicCallback(Runnable callback, double periodSeconds) {
        if (instance == null) {
          return;
        }
    
        instance.addPeriodic(callback, periodSeconds);
      }
}
