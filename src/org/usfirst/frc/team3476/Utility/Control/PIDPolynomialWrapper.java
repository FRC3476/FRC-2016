package org.usfirst.frc.team3476.Utility.Control;

import org.usfirst.frc.team3476.Utility.PolynomialFunction;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

public class PIDPolynomialWrapper implements PIDSource
{
	private PolynomialFunction func;
	private PIDSource source;
	
	public PIDPolynomialWrapper(PolynomialFunction func, PIDSource source)
	{
		this.func = func;
		this.source = source;
	}
	
	@Override
	public void setPIDSourceType(PIDSourceType pidSource) {}

	@Override
	public PIDSourceType getPIDSourceType() {return PIDSourceType.kRate;}

	@Override
	public double pidGet()
	{
		return func.calc(source.pidGet());
	}

}
