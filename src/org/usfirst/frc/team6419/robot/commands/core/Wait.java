package org.usfirst.frc.team6419.robot.commands.core;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class Wait extends Command {
	
	private double _time, endTime;

    public Wait(double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	_time = time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	endTime = Timer.getFPGATimestamp() + _time;
    	System.out.print("Waiting until ");
    	System.out.print(endTime);
    	System.out.print(" (current: ");
    	System.out.print(Timer.getFPGATimestamp());
    	System.out.println(")");
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
