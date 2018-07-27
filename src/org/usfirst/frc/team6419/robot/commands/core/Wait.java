package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * A simple command that waits for the specified time
 */
public class Wait extends Command {
	
	private double _time, endTime;

	/**
	 * Construct a Wait command
	 * @param time time, in seconds, to wait
	 */
    public Wait(double time) {
    	// Store it for later
    	_time = time;
    }

    protected void initialize() {
    	// Get the timestamp now instead of in the constructor because
    	// initialize is normally called some time later
    	endTime = Timer.getFPGATimestamp() + _time;
    	Robot.log(this, String.format("Waiting until %f (current is %f)", endTime, Timer.getFPGATimestamp()));
    }

    protected void execute() {
    	
    }

    protected boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    protected void end() {
    	Robot.log(this, "Wait complete");
    }
    
    protected void interrupted() {
    }
}
