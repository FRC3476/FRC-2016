package org.usfirst.frc.team3476.Utility.Control;

import edu.wpi.first.wpilibj.PIDOutput;

public class PIDOutputWrapper implements PIDOutput
{
	private double out;
	private boolean invert;
	
	public PIDOutputWrapper(boolean invertin)
	{
		out = 0;
		invert = invertin;
	}
	
	@Override
	public void pidWrite(double output)
	{
		out = output;
	}
	
	public double getOutput()
	{
		return invert ? -out : out;
	}
}
