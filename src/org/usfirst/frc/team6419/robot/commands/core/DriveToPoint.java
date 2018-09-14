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
	private boolean finished;
	private boolean atTarget;
	private double timestamp;

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
    	Robot.log(this, "angle: " + theta);
    	dist = Math.hypot(_x, _y);
    	Robot.log(this, "distance: " + dist);
    	Robot.imu.reset();
    	Robot.drivetrain.setTargetHeading(theta);
    	Robot.drivetrain.resetEncoders();
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	Robot.log(this, "Stage is: " + stage);
    	if (stage == 0) {
    		if (!Robot.drivetrain.targetHeadingReached()) {
    			Robot.log(this, "turning");
    			Robot.drivetrain.drive(0, 0, 0);
    			atTarget = false;
    		} else {
    			if (!atTarget) {
    				atTarget = true;
    				timestamp = Timer.getFPGATimestamp();
    			}
    			if (Timer.getFPGATimestamp() - timestamp > 1) {
    				Robot.log(this, "Turn complete");
    				Robot.log(this, "angle is: " +Robot.imu.getHeading());
    				endTime = Timer.getFPGATimestamp() + 1;
    				stage = 1;
    				Robot.drivetrain.setTurningPidEnabled(false);
    				Robot.drivetrain.stop();
    			}
    		}
    	} else if (Timer.getFPGATimestamp() >= endTime) {
    		if (stage == 1) {
    			// Resetting the IMU might make the Talons happy?
    			Robot.imu.reset();
				Robot.log(this, "Resetting IMU");
				stage = 2;
				endTime++;
    		} else if (stage == 2) {
    			Robot.log(this, "Driving forwards");
				Robot.log(dist * Config.ticksPerInch);
				stage = 3;
				Robot.drivetrain.resetEncoders();
				Robot.drivetrain.setFLTarget(dist * Config.ticksPerInch);
				Robot.drivetrain.setFRTarget(dist * Config.ticksPerInch);
				Robot.drivetrain.setBLTarget(dist * Config.ticksPerInch);
				Robot.drivetrain.setBRTarget(dist * Config.ticksPerInch);
    		}
    	} else if (stage == 3) {
    		Robot.log(this,"Speed (L, R): "  +(Robot.drivetrain.getFLSpeed() - Robot.drivetrain.getFRSpeed()));
    		//Robot.drivetrain.drive(0, 0, 0);
    		if (Robot.drivetrain.flTargetReached() &&
        		Robot.drivetrain.frTargetReached() &&
        		Robot.drivetrain.blTargetReached() &&
        		Robot.drivetrain.brTargetReached()) {
    			finished = true;
    		}
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        if (stage == 3 && finished) {
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
