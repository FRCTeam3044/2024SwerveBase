package frc.lib.subsystem;

import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.subsystem.selfcheck.*;
import frc.robot.Robot;
import java.util.ArrayList;
import java.util.List;

public abstract class AdvancedSubsystem extends SubsystemBase {
  public enum SystemStatus {
    OK,
    WARNING,
    ERROR
  }

  private final List<SubsystemFault> faults = new ArrayList<>();
  public static final List<SelfChecking> hardware = new ArrayList<>();
  private final String statusTable;
  private final boolean checkErrors;

  public AdvancedSubsystem() {
    this.statusTable = "SystemStatus/" + this.getName();
    Command systemCheck = getSystemCheckCommand();
    systemCheck.setName(getName() + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
    SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
    checkErrors = RobotBase.isReal();

    setupCallbacks();
  }

  public AdvancedSubsystem(String name) {
    this.setName(name);
    this.statusTable = "SystemStatus/" + name;
    Command systemCheck = getSystemCheckCommand();
    systemCheck.setName(getName() + "Check");
    SmartDashboard.putData(statusTable + "/SystemCheck", systemCheck);
    SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
    checkErrors = RobotBase.isReal();

    setupCallbacks();
  }

  public Command getSystemCheckCommand() {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              SmartDashboard.putBoolean(statusTable + "/CheckRan", false);
              clearFaults();
              publishStatus();
            }),
        systemCheckCommand(),
        Commands.runOnce(
            () -> {
              publishStatus();
              SmartDashboard.putBoolean(statusTable + "/CheckRan", true);
            }));
  }

  private void setupCallbacks() {
    Robot.addPeriodicCallback(this::checkForFaults, 0.25);
    Robot.addPeriodicCallback(this::publishStatus, 1.0);
  }

  private void publishStatus() {
    SystemStatus status = getSystemStatus();
    SmartDashboard.putString(statusTable + "/Status", status.name());
    SmartDashboard.putBoolean(statusTable + "/SystemOK", status == SystemStatus.OK);

    String[] faultStrings = new String[this.faults.size()];
    for (int i = 0; i < this.faults.size(); i++) {
      SubsystemFault fault = this.faults.get(i);
      faultStrings[i] = String.format("[%.2f] %s", fault.timestamp, fault.description);
    }
    SmartDashboard.putStringArray(statusTable + "/Faults", faultStrings);

    if (faultStrings.length > 0) {
      SmartDashboard.putString(statusTable + "/LastFault", faultStrings[faultStrings.length - 1]);
    } else {
      SmartDashboard.putString(statusTable + "/LastFault", "");
    }
  }

  protected void addFault(SubsystemFault fault) {
    if (!this.faults.contains(fault)) {
      this.faults.add(fault);
    }
  }

  protected void addFault(String description, boolean isWarning) {
    this.addFault(new SubsystemFault(description, isWarning));
  }

  protected void addFault(String description, boolean isWarning, boolean sticky) {
    this.addFault(new SubsystemFault(description, isWarning, sticky));
  }

  protected void addFault(String description) {
    this.addFault(description, false);
  }

  public List<SubsystemFault> getFaults() {
    return this.faults;
  }

  public void clearFaults() {
    this.faults.clear();
  }

  public SystemStatus getSystemStatus() {
    SystemStatus worstStatus = SystemStatus.OK;
    for (SubsystemFault f : this.faults) {
      if (f.sticky || f.timestamp > Timer.getFPGATimestamp() - 10) {
        if (f.isWarning) {
          if (worstStatus != SystemStatus.ERROR) {
            System.out.println("warning bad error");
            worstStatus = SystemStatus.WARNING;
          }
        } else {
          System.out.println("bad bad bad error");
          worstStatus = SystemStatus.ERROR;
        }
      }
    }
    System.out.println("good error");
    return worstStatus;
  }

  public void registerHardware(String label, BaseMotorController phoenixMotor) {
    hardware.add(new SelfCheckingPhoenixMotor(label, phoenixMotor));
  }

  public void registerHardware(String label, PWMMotorController pwmMotor) {
    hardware.add(new SelfCheckingPWMMotor(label, pwmMotor));
  }

  public void registerHardware(String label, CANSparkMax spark) {
    hardware.add(new SelfCheckingSparkMax(label, spark));
  }

  // Command to run a full systems check
  protected abstract Command systemCheckCommand();

  // Method to check for faults while the robot is operating normally
  private void checkForFaults() {
    if (checkErrors) {
      for (SelfChecking device : hardware) {
        for (SubsystemFault fault : device.checkForFaults()) {
          addFault(fault);
        }
      }
    }
  }
}
