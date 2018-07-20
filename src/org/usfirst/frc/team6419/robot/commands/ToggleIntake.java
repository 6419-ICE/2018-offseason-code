package org.usfirst.frc.team6419.robot.commands;

public class ToggleIntake extends Command{
	
	public ToggleIntake(){
		requires(Robot.clawPneumatics);
	}
	@Override
	public void initialize(){

	}
	@Override
	public void execute(){
		Robot.clawPneumatics.toggle();
	}
	@Override
	public boolean isFinished(){
		return true;
	}
	@Override
	public void stop(){
	}
	
	@Override
	public void end(){
	}
	@Override
	public void interrupted(){
	}


}