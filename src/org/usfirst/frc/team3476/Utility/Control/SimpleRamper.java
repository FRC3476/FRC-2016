package org.usfirst.frc.team3476.Utility.Control;


public class SimpleRamper extends Ramper
{
	private double rate;//units/second

	public SimpleRamper(double rate)
	{
		this(rate, true, 0);
	}
	
	public SimpleRamper(double rate, boolean unitspersecond)
	{
		this(rate, unitspersecond, 0);
	}
	
	public SimpleRamper(double rate, boolean unitspersecond, double startval)
	{
		super(startval);
		if(unitspersecond)
		{
			this.rate = rate;
		}
		else
		{
			this.rate = 1/rate;
		}
	}
	
	public void calc(double target)
	{
		//get the time diff in ns, to sec, time units per sec to get units
		double timeDiff;
		if(isFirst())
		{
			timeDiff = 1/1E9;
			setFirst(false);
		}
		else
		{
			timeDiff = (System.nanoTime() - getLastTime())/1E9;
		}
		setLastTime(System.nanoTime());
		double deltaTarget = target - getRampedval();
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
			setRampedval(target);
		}
		else//move by the allowed amount
		{
			addToRampedval(delta);
		}
	}

	public void setRate(double rate)
	{
		this.rate = rate;
	}
}
