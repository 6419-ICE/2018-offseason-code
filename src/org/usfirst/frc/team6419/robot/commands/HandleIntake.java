package org.usfirst.frc.team6419.robot.commands;

import org.usfirst.frc.team6419.robot.Robot;

import edu.wpi.first.wpilibj.command.Command;

/**
 * Default command for the {@link Intake org.usfirst.frc.team6419.robot.subsystems.Intake}
 * 
 * All it does is stop the intake motors.
 */
public class HandleIntake extends Command {

    public HandleIntake() {
    	requires(Robot.intake);
    }

    protected void initialize() {
    	Robot.intake.setPower(0);
    }
    
    protected void execute() {
    	
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
    	
    }

    protected void interrupted() {
    	
    }
}
