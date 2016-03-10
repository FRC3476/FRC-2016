package org.usfirst.frc.team3476.Utility.Control;


public class Ramper
{
	private double rate;//units/second
	private double rampedval;
	private long lastTime;
	private boolean first;

	public Ramper(double rate)
	{
		this(rate, true, 0);
	}
	
	public Ramper(double rate, boolean unitspersecond)
	{
		this(rate, unitspersecond, 0);
	}
	
	public Ramper(double rate, boolean unitspersecond, double startval)
	{
		if(unitspersecond)
		{
			this.rate = rate;
		}
		else
		{
			this.rate = 1/rate;
		}
		rampedval = startval;
		lastTime = System.nanoTime();
		first = true;
	}
	
	public double doubleAction(double target)
	{
		calc(target);
		return rampedval;
	}
	
	public double get()
	{
		return rampedval;
	}
	
	public void calc(double target)
	{
		//get the time diff in ns, to sec, time units per sec to get units
		double timeDiff;
		if(first)
		{
			timeDiff = 1/1E9;
			lastTime = System.nanoTime();
			first = false;
		}
		else
		{
			timeDiff = (System.nanoTime() - lastTime)/1E9;
			lastTime = System.nanoTime();
		}
		double deltaTarget = target - rampedval;
		int dir;
		if(deltaTarget != 0)
		{
			dir = deltaTarget < 0 ? -1 : 1;
		}
		else
		{
			dir = 0;
		}
		double delta = rate*timeDiff*dir;
		
		double checkDelta = delta - deltaTarget;
		if(checkDelta*delta > 0)//signs the same, target too close, set to target - end case
		{
			rampedval = target;
		}
		else//move by the allowed amount
		{
			rampedval = rampedval + delta;
		}
	}
	
	public void reset()
	{
		first = true;
	}
}
