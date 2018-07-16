package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Config;
import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class StrafeToPoint extends Command {
	
	private double x, y, timeRemaining, deltaT, lastTimestamp;
	private double[] speeds;

    public StrafeToPoint(double x, double y) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	this.x = x;
    	this.y = y;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	timeRemaining = Math.hypot(x, y) / Config.stpMaxSpeed;
    	lastTimestamp = Timer.getFPGATimestamp();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	speeds = Robot.drivetrain.driveWithVelocity(x, y);
    	
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    }
}
