package org.usfirst.frc.team3476.ScriptableAuto;

import org.usfirst.frc.team3476.Main.Subsystem;

public class ErrorSystem implements Subsystem
{
	@Override
	public String[] getAutoCommands() {return new String[]{"error"};}
	
	@Override
	public void doAuto(double[] params, String command) {}
	
	@Override
	public boolean isAutoDone() {return true;}
	
	@Override
	public String[] getConstantRequest(){return new String[]{};}
	
	@Override
	public void returnConstantRequest(double[] constants){}
	
	@Override
	public String toString()
	{
		return "ErrorSystem";
	}

	@Override
	public void update(){}

	@Override
	public void stopThreads(){}

	@Override
	public void startThreads(){}

	@Override
	public void terminateThreads(){}

	@Override
	public void end() {}
}
