package org.usfirst.frc.team3476.Subsystems;

import org.usfirst.frc.team3476.Main.Subsystem;

public class Test implements Subsystem
{
	double test1, test2;
	
	public String[] getAutoCommands(){return new String[]{"test"};}
	
	public void doAuto(double[] params, String command)
	{
		if(command.equals("test"))
		{
			System.out.println("Test echo: colon param: " + params[0] + " @ param: " + params[1] + ". Constants: " + test1 + ", " + test2);
		}
	}
	
	public boolean isAutoDone(){return true;}
	
	public String[] getConstantRequest(){return new String[]{"test1", "test2"};}//Request all needed constants
	
	public void returnConstantRequest(double[] constants)
	{
		test1 = constants[0];
		test2 = constants[1];
	}//Request all needed constants
	
	public String toString()
	{
		return "Shooter";
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
