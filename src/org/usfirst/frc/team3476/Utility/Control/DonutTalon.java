package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;

public class DonutTalon extends Talon
{
	private double donutThreshold;
	private double clamp;
	private boolean scale;
	
	public DonutTalon(int deviceNumber, double donutThresholdin, double clampin, boolean scale)
	{
		super(deviceNumber);
		this.donutThreshold = Math.abs(donutThresholdin);
		this.clamp = Math.abs(clampin);
		this.scale = scale;
	}
	
	public DonutTalon(int deviceNumber, double donutThresholdin, double clampin)
	{
		this(deviceNumber, donutThresholdin, clampin, false);
	}
	
	public DonutTalon(int deviceNumber, double donutThresholdin, boolean scale)
	{
		this(deviceNumber, donutThresholdin, 1, scale);
	}
	
	public DonutTalon(int deviceNumber, double donutThresholdin)
	{
		this(deviceNumber, donutThresholdin, 1, false);
	}
	
	public void setDonutThreshold(double donutThreshold)
	{
		this.donutThreshold = donutThreshold;
	}
	
	public void setScale(boolean scale)
	{
		this.scale = scale;
	}

	public void setClamp(double clamp)
	{
		this.clamp = clamp;
	}
	
	@Override
	public void set(double value)
	{
		if(scale)
		{
			super.set(OrangeUtility.scalingDonut(value, donutThreshold, clamp, 1));//we know the max(min) is 1 cause this is a motor
		}
		else
		{
			super.set(OrangeUtility.coerce(OrangeUtility.donut(value, donutThreshold), clamp, -clamp));
		}
	}
}
