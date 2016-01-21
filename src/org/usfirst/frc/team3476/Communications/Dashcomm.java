package org.usfirst.frc.team3476.Communications;

import java.util.ArrayList;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashcomm
{

public static final String TARGETDELIMITER = "\t", DATADELIMETER = " ";

	public static double[][] getTargetData(String key)
	{
		ArrayList<Double[]> result = new ArrayList<Double[]>();
		String stringData = SmartDashboard.getString(key);
		String[] targets = stringData.split(TARGETDELIMITER);
		for(String target : targets)
		{
			String[] manyData = target.split(DATADELIMETER);
			ArrayList<Double> targetData = new ArrayList<Double>();
			for(String dataPiece : manyData)
			{
				double data = OrangeUtility.cleanDoubleParse(dataPiece);
				targetData.add(data);
			}
			result.add((Double[])targetData.toArray());
		}
		Double[][] objtemp = (Double[][])result.toArray();
		double[][] pritemp = new double[objtemp.length][objtemp[0].length];
		for(int i = 0; i < objtemp.length; i++)
		{
			for(int j = 0; j < objtemp[i].length; j++)
			{
				pritemp[i][j] = objtemp[i][j];
			}
		}
		return pritemp;
	}
}