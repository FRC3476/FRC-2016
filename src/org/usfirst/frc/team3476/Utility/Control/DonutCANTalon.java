package org.usfirst.frc.team3476.Utility.Control;

import java.util.Arrays;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.CANTalon;

public class DonutCANTalon extends CANTalon
{
	private double donutThreshold;
	private double clamp;
	
	public DonutCANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs, double donutThresholdin, double clampin)
	{
		super(deviceNumber, controlPeriodMs, enablePeriodMs);
		donutThreshold = Math.abs(donutThresholdin);
		clamp = Math.abs(clampin);
	}
	
	public DonutCANTalon(int deviceNumber, int controlPeriodMs, int enablePeriodMs, double donutThresholdin)
	{
		this(deviceNumber, controlPeriodMs, enablePeriodMs, donutThresholdin, 1);
	}
	
	public DonutCANTalon(int deviceNumber, int controlPeriodMs, double donutThresholdin, double clampin)
	{
		super(deviceNumber, controlPeriodMs);
		donutThreshold = Math.abs(donutThresholdin);
		clamp = Math.abs(clampin);
	}
	
	public DonutCANTalon(int deviceNumber, int controlPeriodMs, double donutThresholdin)
	{
		this(deviceNumber, controlPeriodMs, donutThresholdin, 1);
	}
	
	public DonutCANTalon(int deviceNumber, double donutThresholdin, double clampin)
	{
		super(deviceNumber);
		donutThreshold = Math.abs(donutThresholdin);
		clamp = Math.abs(clampin);
	}
	
	public DonutCANTalon(int deviceNumber, double donutThresholdin)
	{
		this(deviceNumber, donutThresholdin, 1);
	}
	
	public void setDonutThreshold(double donutThreshold)
	{
		this.donutThreshold = donutThreshold;
	}
	
	public void setClamp(double clamp)
	{
		this.clamp = clamp;
	}
	
	@Override
	public void set(double value)
	{
		/*if(value == 0)
		{
			StackTraceElement[] elements = Thread.getAllStackTraces().get(Thread.currentThread());
			System.out.println(Arrays.toString(elements));
		}*/
		super.set(OrangeUtility.coerce(OrangeUtility.donut(value, donutThreshold), clamp, -clamp));
	}
}
