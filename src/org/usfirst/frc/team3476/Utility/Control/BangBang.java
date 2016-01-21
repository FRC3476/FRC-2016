package org.usfirst.frc.team3476.Utility.Control;

public class BangBang extends ControlLoop
{
	public BangBang(double[] outputrangein)
	{
		super(outputrangein);
	}

	@Override
	protected double run(double process)
	{
		// TODO Auto-generated method stub
		if(process < getSetpoint())
		{
			return Math.max(getOutputrange()[0], getOutputrange()[1]);
		}
		if(process > getSetpoint())
		{
			return Math.min(getOutputrange()[0], getOutputrange()[1]);
		}
		return 0;
	}
}
