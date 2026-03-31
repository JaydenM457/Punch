package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.HopperSubsytem;

public class PulseCommand extends Command {

    private HopperSubsytem Hopper;
    private double speed = -1.0;      // start at full reverse
    private boolean goingUp = true;   // direction flag

    public PulseCommand(HopperSubsytem Hopper) {
        this.Hopper = Hopper;
        addRequirements(Hopper); // make sure the command claims the subsystem
    }

    @Override
    public void initialize() {
        // nothing special needed here
    }

    @Override
    public void execute() {
        // --- smooth up/down logic ---
        if (goingUp) {
            speed += 0.05;         // ramp up (toward -0.5)
            if (speed >= -0.5) goingUp = false;
        } else {
            speed -= 0.05;         // ramp down (toward -1.0)
            if (speed <= -1.0) goingUp = true;
        }

        // apply the speed to the hopper
        Hopper.set(speed);
    }

    @Override
    public boolean isFinished() {
        return false;  // run until manually canceled
    }

    @Override
    public void end(boolean interrupted) {
        Hopper.set(0);  // stop hopper when done
    }
}
