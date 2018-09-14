package org.usfirst.frc.team6419.robot.core;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public abstract class TimeoutCommand extends Command {
	
	private double timeout, endTime;

    public TimeoutCommand(double time) {
    	timeout = time;
    }
    
    public void initialize() {
    	endTime = Timer.getFPGATimestamp() + timeout;
    }
    
    protected abstract boolean isDone();
    
    public boolean isFinished() {
    	return Timer.getFPGATimestamp() >= endTime || isDone();
    }
    
    protected double getRuntime() {
    	return Timer.getFPGATimestamp() - (endTime - timeout);
    }
}
