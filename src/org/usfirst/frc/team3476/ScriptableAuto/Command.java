package org.usfirst.frc.team3476.ScriptableAuto;

/**
 * Represents a single autonomous command.
 * @author Anthony Demetrescu
 *
 */
public class Command
{
	String command;
	double[] params;
	String endCondition;
	double endValue;
	boolean started;
	
	/**
	 * Constructs a Command with the given parameters.
	 * @param commandin the literal command (name)
	 * @param paramsin the parameter values that subsystems may use as they see fit
	 * @param endCon the end condition for a command
	 * @param endVal the value related to the end condition
	 */
	public Command(String commandin, double[] paramsin, String endCon, double endVal)
	{
		command = commandin;
		params = paramsin;
		endCondition = endCon;
		endValue = endVal;
		started = false;
	}
	
	/**
	 * Constructs a Command with the given parameters.
	 * @param commandin the literal command (name)
	 * @param paramsin the parameter values that subsystems may use as they see fit
	 */
	public Command(String commandin, double[] paramsin)
	{
		command = commandin;
		params = paramsin;
		endCondition = "none";
		endValue = 0.0;
		started = false;
	}
	
	/**
	 * Constructs a Command with the given parameters.
	 * @param commandin the literal command (name)
	 */
	public Command(String commandin)
	{
		command = commandin;
		params = new double[1];
		endCondition = "none";
		endValue = 0.0;
		started = false;
	}
	
	/**
	 * @return the name of this command.
	 */
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
