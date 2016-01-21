package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

public abstract class ControlLoop
{
	private double setpoint;
	private double[] outputrange;
	
	public ControlLoop(double[] outputrangein)
	{
		setpoint = 0;
		outputrange = outputrangein;
	}
	
	public void setSetpoint(double setpointin)
	{
		setpoint = setpointin;
	}
	
	public double output(double process)
	{
		return OrangeUtility.coerce(run(process), Math.max(outputrange[0], outputrange[1]), Math.min(outputrange[0], outputrange[1]));
	}
	
	public double getSetpoint()
	{
		return setpoint;
	}
	
	public double[] getOutputrange()
	{
		return outputrange;
	}
	
	public void setOutputrange(double[] outputrangein)
	{
		outputrange = outputrangein;
	}
	
	protected abstract double run(double process);
	
	public double getError(double process)
	{
		return setpoint - process;
	}
}
