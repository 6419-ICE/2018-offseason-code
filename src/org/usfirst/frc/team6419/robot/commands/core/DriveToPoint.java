package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Config;
import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPoint extends Command {
	
	private double _x, _y, theta, dist;
	private int stage;

    public DriveToPoint(double x, double y) {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.drivetrain);
    	_x = x;
    	_y = y;
    	stage = 0;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	theta = Math.atan2(_y, _x) - Math.PI / 2.0;
    	dist = Math.hypot(_x, _y);
    	Robot.drivetrain.setTargetHeading(theta);
    	Robot.drivetrain.resetEncoders();
    	Robot.drivetrain.setSpeedLimit(0.5);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (stage == 0) {
    		if (!Robot.drivetrain.targetHeadingReached()) {
    			System.out.println("Turning");
    			Robot.drivetrain.drive(0, 0, 0);
    		} else {
    			System.out.println("Set");
    			System.out.println(dist * Config.ticksPerInch);
    			stage = 1;
    			Robot.drivetrain.setFLTarget(dist * Config.ticksPerInch);
    			Robot.drivetrain.setFRTarget(dist * Config.ticksPerInch);
    			Robot.drivetrain.setBLTarget(dist * Config.ticksPerInch);
    			Robot.drivetrain.setBRTarget(dist * Config.ticksPerInch);
    		}
    	} else {
    		System.out.println("Driving");
    		//Robot.drivetrain.drive(0, 0, 0);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (stage == 1 &&
        		Robot.drivetrain.flTargetReached() &&
        		Robot.drivetrain.frTargetReached() &&
        		Robot.drivetrain.blTargetReached() &&
        		Robot.drivetrain.brTargetReached()) {
        	System.out.println("Finished");
        	return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.setSpeedLimit(1);
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
