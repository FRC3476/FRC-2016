package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.SpeedController;

public class PIDMotorGroup implements PIDOutput
{
	private SpeedController[] controllers;
	private int[] dirs;
	
	/**
	 * Don't use, hasn't been implemented.
	 */
	public PIDMotorGroup(SpeedController... speedControllers){}
	
	public PIDMotorGroup(SpeedController[] controllers, int[] dirs)
	{
		this.controllers = controllers;
		for(int i = 0; i < dirs.length; i++)
		{
			if(dirs[i] > 1)
			{
				dirs[i] = 1;
			}
			else if(dirs[i] < -1)
			{
				dirs[i] = -1;
			}
		}
		this.dirs = dirs;
	}
	
	public void setDirs(int[] dirs)
	{
		this.dirs = dirs;
	}
	
	@Override
	public void pidWrite(double output)
	{
		for(int i = 0; i < controllers.length; i++)
		{
			controllers[i].set(output*dirs[i]);
		}
	}
	
	public double[] get()
	{
		double[] values = new double[controllers.length];
		for(int i = 0; i < values.length; i++)
		{
			values[i] = controllers[i].get()*dirs[i]*(controllers[i].getInverted() ? -1 : 1);
		}
		return values;
	}

}
