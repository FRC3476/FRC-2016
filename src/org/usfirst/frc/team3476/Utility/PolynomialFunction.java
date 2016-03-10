package org.usfirst.frc.team3476.Utility;

import java.util.Arrays;

public class PolynomialFunction
{
	private double[] coefficients;
	
	public PolynomialFunction(double... coefficients)
	{
		this.coefficients = Arrays.copyOf(coefficients, coefficients.length);
	}
	
	public double calc(double x)
	{
		double result = 0;
		
		for(int order = 0; order < coefficients.length; order++)
		{
			result += coefficients[order]*Math.pow(x, order);
		}
		
		return result;
	}
}
