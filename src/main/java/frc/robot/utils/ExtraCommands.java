package frc.robot.utils;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ExtraCommands {
    /**
     * Generate a command that runs code until a condition is met
     * 
     * @param run          The code to run
     * @param isFinished   The condition to check
     * @param requirements The subsystems required by the command
     * @return The command
     */
    public static Command runUntil(Runnable run, BooleanSupplier isFinished, Subsystem... requirements) {
        return new FunctionalCommand(() -> {
        }, run, (interrupted) -> {
        }, isFinished, requirements);
    }
}
