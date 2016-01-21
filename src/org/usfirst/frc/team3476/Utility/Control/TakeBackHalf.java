package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.Timer;

public class TakeBackHalf extends ControlLoop
{
	private double lastTBH, integral, lastTime, lastProcess, gain, MAX;
	private Timer timer;
	
	
	public TakeBackHalf(double[] outputrangein, double gainin, double MAXIN)
	{
		super(outputrangein);
		lastTBH = 0;
		integral = 0;
		gain = gainin;
		MAX = MAXIN;
		
		timer = new Timer();
		timer.start();
	}
	
	public void setGain(double gainin)
	{
		gain = gainin;
	}

	@Override
	protected double run(double process)
	{
		process = OrangeUtility.normalize(process, MAX, 0, 1, 0);
		double setpoint = getSetpoint();
		double curTime = timer.get();
		
		integral = OrangeUtility.coerce(integral + (curTime - lastTime)*(setpoint - process)*gain, Math.max(getOutputrange()[0], getOutputrange()[1]), Math.min(getOutputrange()[0], getOutputrange()[1]));
		if((setpoint - lastProcess)*(setpoint - process) < 0)//Change of signs means overshoot
		{
			integral = (lastTBH + integral)/2;
			lastTBH = integral;
		}
		
		lastTime = curTime;
		lastProcess = process;
		return integral;
	}
	
	@Override
	public void setSetpoint(double setpointin)
	{
		super.setSetpoint(OrangeUtility.normalize(setpointin, MAX, 0, 1, 0));
		lastTBH = 2*setpointin - 1;
	}
	
	public double getMax()
	{
		return MAX;
	}
}
