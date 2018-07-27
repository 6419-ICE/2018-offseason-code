package org.usfirst.frc.team6419.robot.commands.core;


import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

/**
 * Drive straight for the specified time
 */
public class TimedDriveStraight extends Command {
	
	private double time, endTime;
	
	/**
	 * Construct a TimedDriveStraight command
	 * @param time time, in seconds, to drive
	 */
    public TimedDriveStraight(double time) {
    	requires(Robot.drivetrain);
    	this.time = time;
    }

    protected void initialize() {
    	endTime = Timer.getFPGATimestamp() + time;
    	// Activate the drivetrain's turning PID to mitigate drift while driving
    	Robot.drivetrain.setTargetHeading(Robot.imu.getHeading());
    }

    protected void execute() {
    	Robot.drivetrain.drive(0, -0.5, 0);
    }

    protected boolean isFinished() {
        return Timer.getFPGATimestamp() >= endTime;
    }

    protected void end() {
    	Robot.drivetrain.stop();
    }

    protected void interrupted() {
    	end();
    }
}
