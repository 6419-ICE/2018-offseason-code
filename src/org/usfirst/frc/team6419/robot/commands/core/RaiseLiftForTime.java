package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class RaiseLiftForTime extends Command {
	
	private double endTime;

    public RaiseLiftForTime(double time) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.lift);
    	endTime = Timer.getFPGATimestamp() + time;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.lift.setPower(0.555);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.log(this, "Lifting Lift");
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.lift.park();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
