package org.usfirst.frc.team6419.robot.commands.core;

import org.usfirst.frc.team6419.robot.Config;
import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class DriveToPoint extends Command {
	
	private double _x, _y, theta, dist;
	private int stage;
	private double endTime;

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
    	theta = -(Math.atan2(_y, _x) - Math.PI / 2.0);
    	dist = Math.hypot(_x, _y);
    	Robot.imu.reset();
    	Robot.drivetrain.setTargetHeading(theta);
    	Robot.drivetrain.resetEncoders();
    	Robot.log(this, theta);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (stage == 0) {
    		if (!Robot.drivetrain.targetHeadingReached()) {
    			Robot.drivetrain.drive(0, 0, 0);
    		} else {
    			endTime = Timer.getFPGATimestamp() + 2;
    			stage = 1;
    			Robot.drivetrain.setTurningPidEnabled(false);
    		}
    	} else if (Timer.getFPGATimestamp() >= endTime && stage != 2) {
    		Robot.log(this, "Set");
			Robot.log(dist * Config.ticksPerInch);
			stage = 2;
		// Resetting the IMU might make the Talons happy?
		Robot.imu.reset();
			Robot.drivetrain.setFLTarget(dist * Config.ticksPerInch);
			Robot.drivetrain.setFRTarget(dist * Config.ticksPerInch);
			Robot.drivetrain.setBLTarget(dist * Config.ticksPerInch);
			Robot.drivetrain.setBRTarget(dist * Config.ticksPerInch);
    	} else {
    		Robot.log(this, Robot.drivetrain.getFLSpeed() - Robot.drivetrain.getFRSpeed());
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
        	Robot.log(this, "Finished");
        	return true;
        }
        return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	Robot.drivetrain.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}
