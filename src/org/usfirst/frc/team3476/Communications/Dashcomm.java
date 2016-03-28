package org.usfirst.frc.team3476.Communications;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

/**
 * Parses NetworkTables data into usable form for the robot.
 * @author Anthony Demetrescu
 *
 */
public class Dashcomm
{

public static final String TARGETDELIMITER = "\t", DATADELIMETER = " ";
public static final String TARGETINGKEY = "data/camera";
public static final int NUMDATAPIECES = 3;

	/**
	 * Retrieves the data from vision processing about target locations.
	 * @return the array containing targets and their associated values (indexed as: [target][XPOS, YPOS, AREA])
	 */
	public static double[][] getTargetData()
	{
		double[] rawData = get(TARGETINGKEY, new double[]{Double.NaN, Double.NaN, Double.NaN});
		//System.out.println("RawData: " + Arrays.toString(rawData));
		if(rawData.length == 0)
			return new double[][]{{Double.NaN, Double.NaN, Double.NaN}};
		int numtargets = rawData.length/3;
		
		double[][] result = new double[numtargets][NUMDATAPIECES];
		for(int i = 0; i < numtargets; i++)
		{
			for(int j = 0; j < NUMDATAPIECES; j++)
			{
				result[i][j] = rawData[3*i + j];
			}
		}
		return result;
	}
		
	//*******************************
	//************GETTERS************
	//*******************************
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed double value, if available
	 */
	public static double get(String keypath, double defaultvalue)
	{
		return NetworkTable.getTable("").getNumber(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed double[] value, if available
	 */
	public static double[] get(String keypath, double[] defaultvalue)
	{
		return NetworkTable.getTable("").getNumberArray(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed boolean value, if available
	 */
	public static boolean get(String keypath, boolean defaultvalue)
	{
		return NetworkTable.getTable("").getBoolean(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed boolean[] value, if available
	 */
	public static boolean[] get(String keypath, boolean[] defaultvalue)
	{
		return NetworkTable.getTable("").getBooleanArray(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed String value, if available
	 */
	public static String get(String keypath, String defaultvalue)
	{
		return NetworkTable.getTable("").getString(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed String[] value, if available
	 */
	public static String[] get(String keypath, String[] defaultvalue)
	{
		return NetworkTable.getTable("").getStringArray(keypath, defaultvalue);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param defaultvalue the default value if no keyed value is available
	 * @return the keyed Object[] value, if available
	 */
	public static Object get(String keypath, Object defaultvalue)
	{
		return NetworkTable.getTable("").getValue(keypath, defaultvalue);
	}
	
	//*******************************
	//************PUTTERS************
	//*******************************
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param value the double value to put
	 */
	public static void put(String keypath, double value)
	{
		NetworkTable.getTable("").putNumber(keypath, value);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param value the double[] value to put
	 */
	public static void put(String keypath, double[] value)
	{
		NetworkTable.getTable("").putNumberArray(keypath, value);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param value the String value to put
	 */
	public static void put(String keypath, String value)
	{
		NetworkTable.getTable("").putString(keypath, value);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param value the String[] value to put
	 */
	public static void put(String keypath, String[] value)
	{
		NetworkTable.getTable("").putStringArray(keypath, value);
	}
	
	/**
	 * Simplifies the Dashboard communication by simply using the exact keypath of the value instead of the table and key.
	 * @param keypath the path to the value including the key - DO NOT PREPEND A '/'
	 * @param value the boolean value to put
	 */
	public static void put(String keypath, boolean value)
	{
		NetworkTable.getTable("").putBoolean(keypath, value);
	}
}