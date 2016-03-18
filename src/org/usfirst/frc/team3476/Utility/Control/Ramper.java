package org.usfirst.frc.team3476.Utility.Control;

public abstract class Ramper
{
	private double rampedval;
	private long lastTime;
	private boolean first;
	
	public Ramper()
	{
		this(0);
	}
	
	public Ramper(double startval)
	{
		rampedval = startval;
		lastTime = System.nanoTime();
		first = true;
	}
	
	abstract void calc(double target);
	
	public double doubleAction(double target)
	{
		calc(target);
		return rampedval;
	}
	
	public double get()
	{
		return rampedval;
	}
	
	public void reset()
	{
		first = true;
	}

	public double getRampedval()
	{
		return rampedval;
	}

	public void setRampedval(double rampedval)
	{
		this.rampedval = rampedval;
	}
	
	public void addToRampedval(double delta)
	{
		this.rampedval += delta;
	}

	public long getLastTime()
	{
		return lastTime;
	}

	public void setLastTime(long lastTime)
	{
		this.lastTime = lastTime;
	}

	public boolean isFirst()
	{
		return first;
	}

	public void setFirst(boolean first)
	{
		this.first = first;
	}
}
