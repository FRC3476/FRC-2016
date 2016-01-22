package org.usfirst.frc.team3476.Communications;

import java.util.ArrayList;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Parses SmartDashboard data into usable form for the robot.
 * @author Anthony Demetrescu
 *
 */
public class Dashcomm
{

public static final String TARGETDELIMITER = "\t", DATADELIMETER = " ";
public static final String TARGETINGKEY = "Camera Data";

	/**
	 * Retrieves the data from vision processing about target locations.
	 * @return the array containing targets and their associated values (indexed as: [target][XPOS, YPOS, AREA])
	 */
	public static double[][] getTargetData()
	{
		ArrayList<Double[]> result = new ArrayList<Double[]>();
		String stringData = SmartDashboard.getString(TARGETINGKEY);
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