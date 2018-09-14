package org.usfirst.frc.team6419.robot.commands;

import java.util.TimerTask;

import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 *
 */
public class IntakeCube extends Command {
	
	private class StopIntakeTask extends TimerTask {
		@Override
		public void run() {
			Robot.intake.setPower(0);
			taskDone = true;
		}
	}
	
	private java.util.Timer timer;
	private boolean taskDone;
	
    public IntakeCube() {
        // Use requires() here to declare subsystem dependencies
        // eg. requires(chassis);
    	requires(Robot.intake);
    	timer = new java.util.Timer();
    	taskDone = false;
    }

    // Called just before this Command runs the first time
    protected void initialize() {
    	Robot.intake.setPower(1);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	if (Robot.intake.getCubeDistance() < 13) {
    		if (!Robot.intake.isClosed()) {
    			Robot.intake.setClosed(true);
    		}
    		Robot.intake.setPower(1);
    	} else if (Robot.intake.getCubeDistance() < 3) {
    		timer.schedule(new StopIntakeTask(), 500);
    	}
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return (Robot.intake.getCubeDistance() > 13 || (Robot.intake.getCubeDistance() < 3 && taskDone)) && !(Robot.m_oi.getRightJoystick().getRawButton(4) || Robot.m_oi.getLeftJoystick().getRawButton(4));
    }

    // Called once after isFinished returns true
    protected void end() {
    	timer.cancel();
    	Robot.intake.setPower(0);
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	// Don't call end
    }
}
