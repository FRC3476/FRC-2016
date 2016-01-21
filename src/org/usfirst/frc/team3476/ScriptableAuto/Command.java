package org.usfirst.frc.team3476.ScriptableAuto;

public class Command
{
	String command;
	double[] params;
	String endCondition;
	double endValue;
	boolean started;
	
	public Command(String commandin, double[] paramsin, String endCon, double endVal)
	{
		command = commandin;
		params = paramsin;
		endCondition = endCon;
		endValue = endVal;
		started = false;
	}
	
	public Command(String commandin, double[] paramsin)
	{
		command = commandin;
		params = paramsin;
		endCondition = "none";
		endValue = 0.0;
		started = false;
	}
	
	public Command(String commandin)
	{
		command = commandin;
		params = new double[1];
		endCondition = "none";
		endValue = 0.0;
		started = false;
	}
	
	public String getName()
	{
		return command;
	}
	
	public double getParam(int index)
	{
		return params[index];
	}
	
	public double[] getParams()
	{
		return params;
	}
	
	public void start()
	{
		started = true;
	}
	
	public boolean isStarted()
	{
		return started;
	}
	
	public String toString()
	{
		String toReturn = command;
		if(params[0] != 0)
		{
			toReturn += ": " + params[0];
		}
		if(params[1] != 0)
		{
			toReturn += " @ " + params[1];
		}
		return toReturn;
	}
}
