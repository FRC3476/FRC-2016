package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

/**
 * Parses a scriptable autonomous script with a constants file.
 * @author Anthony Demetrescu
 *
 */
public class Parser
{
	final String BACKUPCONSTANTS = "//Driving deadzones\nDRIVEDEAD = 2\nDRIVESTRAIGHTDEAD = 5\nTURNDEAD = 5\n\n//encoders\nUSELEFT = 1\nUSERIGHT = 1\n\n//Drivestraight PID\nSTRAIGHTP = 1.5\nSTRAIGHTI = 0\nSTRAIGHTD = 0.001\n\n//Drive PID\nDRIVEP = 1.3\nDRIVEI = 0.005\nDRIVED = 0\n\n//Turn PID\nTURNP = 1.7\nTURNI = 0.005\nTURND = 0\n\n//Intake constants\nSUCKMOTORSPEED = -1\nLOADMOTORSPEED = -1\nFORWARDISDOWN = 0 //false\nAIMUPPOWERED = 1 //true\n\n//Shooter constants\nSHOOTEROUTPUTRANGEHIGH = 1\nSHOOTEROUTPUTRANGELOW = -1\nSHOOTERIGAIN = 0.00001\nFLY1DIR = -1\nFLY2DIR = 1\nFLY3DIR = -1\nFLY4DIR = 1\nGRABFRISBEETIME = 0.65\nSHOOTFRISBEETIME = 0.33\nFLYWHEELDEAD = 100\nFLYWHEELMAXSPEED = 3000";
	
	final String PARALLELSEPARATOR = ";", CONSTANTSSEPERATOR = "\u001B", YEARSEPERATOR = "~", FIRSTPARAM = ":", SECONDPARAM = "@", THEN = ">";
	String script;
	String constants;
	String constantYear;
	
	/**
	 * Constructor that takes a script, constants files String, and a year identifier String.
	 * @param scriptin the autonomous script String
	 * @param constantsin the constants file String
	 * @param thisYear the year identifier String
	 */
	public Parser(String scriptin, String constantsin, String thisYear)
	{
		script = scriptin;
		constantYear = thisYear;
		constants = retrieveThisYear(constantsin);
	}
	
	/**
	 * Returns the next line of the autonomous script.
	 * @return the ArrayList of CommandBlocks that represent this line.
	 */
	public ArrayList<CommandBlock> nextLine()
	{
		if(script.equals(""))
		{
			return new ArrayList<CommandBlock>();
		}
		int endOfLine = script.indexOf("\n");
		if(endOfLine == -1)
		{
			endOfLine = script.length();
		}
		String line = script.substring(0, endOfLine);//Get the next line
		if (script.length() == endOfLine)
		{
			script = "";//Remove the line we just retrieved
		}
		else
		{
			script = script.substring(endOfLine + 1);
		}
		
		//Remove comments
		/*int comdex = line.indexOf("//");
		if(comdex != -1)
		{
			line = line.substring(0, comdex);
		}
		//Obsolete code, here in case new code doesn't work
		*/
		
		//Replacement code
		line = line.split("//")[0];
		
		ArrayList<CommandBlock> lineCommands = new ArrayList<CommandBlock>();
		
		/*int semiIndex = line.indexOf(PARALLELSEPARATOR);
		while(semiIndex != -1)//While there are still parallel commands to be processed, create command blocks
		{
			line = line.trim();
			String commandBlock = line.substring(0, semiIndex);
			line = line.substring(semiIndex + 1);
			
			lineCommands.add(parseCommandBlock(commandBlock));
			
			semiIndex = line.indexOf(PARALLELSEPARATOR);
		}
		lineCommands.add(parseCommandBlock(line));
		//Obsolete code, here in case new code doesn't work
		*/
		
		//Replacement code
		for(String commandBlock : line.split(PARALLELSEPARATOR))
		{
			lineCommands.add(parseCommandBlock(commandBlock));
		}
		
		return lineCommands;
	}
	
	/**
	 * Parses a String into a single CommandBlock.
	 * Deals with whitespace in commandBlock.
	 * Used by nextLine().
	 * @param commandBlock the string to be parsed into a CommandBlock.
	 * @return the CommandBlock representation of the input String
	 */
	private CommandBlock parseCommandBlock(String commandBlock)
	{
		commandBlock = commandBlock.trim();
		ArrayList<Command> blockCommands = new ArrayList<Command>();
		
		/*int thendex = commandBlock.indexOf(THEN);
		while(thendex != -1)
		{
			commandBlock = commandBlock.trim();
			String command = commandBlock.substring(0, thendex);
			commandBlock = commandBlock.substring(thendex + 1);
			
			blockCommands.add(parseCommand(command));
			
			thendex = commandBlock.indexOf(THEN);
		}
		//Obsolete code, here in case new code doesn't work
		*/
		
		//Replacement code
		for(String command : commandBlock.split(THEN))
		{
			blockCommands.add(parseCommand(command));
		}
		
		return new CommandBlock(blockCommands);
	}
	
	/**
	 * Parses a String into a single Command.
	 * Deals with whitespace in thenBlock.
	 * Used by parseCommandBlock().
	 * @param thenBlock the subunit String of a CommandBlock String
	 * @return the Command representation of the input String
	 */
	private Command parseCommand(String thenBlock)
	{
		thenBlock = thenBlock.trim();
		String command = "error";
		double colonParam = 0.0;
		double atParam = 0.0;
		int colonIndex = thenBlock.indexOf(FIRSTPARAM);
		int atIndex = thenBlock.indexOf(SECONDPARAM);
		if(colonIndex != -1)//Is there a colon?
		{
			command = thenBlock.substring(0, colonIndex).trim();
			if(atIndex != -1)//Is there an at?
			{
				colonParam = OrangeUtility.cleanDoubleParse(thenBlock.substring(colonIndex + 1, atIndex)); //Grab the stuff between the : and @ and parse
				atParam = OrangeUtility.cleanDoubleParse(thenBlock.substring(atIndex + 1)); //Grab the stuff after the @ and parse
			}
			else
			{
				colonParam = OrangeUtility.cleanDoubleParse(thenBlock.substring(colonIndex + 1)); //Grab the stuff after the : - no @
			}
		}
		else
		{
			command = thenBlock;
		}
		
		//Construct command from parsed command block
		double[] params = new double[2];
		params[0] = colonParam;
		params[1] = atParam;
		return new Command(command, params);
	}
	
	/**
	 * Retrieves the constant value with the specified key from the constants file for the Parser's year.
	 * @param key the key to search for
	 * @return the value associated with the key
	 * @throws IOException
	 */
	public double getConstant(String key) throws IOException
	{
		Matcher match = Pattern.compile("^" + key + "\\s*=", Pattern.MULTILINE).matcher(constants);
		
		int keydex = match.find() ? match.end() : -1;
		
		if(keydex == -1)
		{
			System.out.println("Match not found for key " + key);
			//throw new IOException("Missing key \"" + key + "\" in constants file: \"" + constantYear + "\"\nConstants in use: \"" + constants + "\"");
		}
//		if(equals == -1)
//		{
//			throw new IOException("Missing equals in " + constantYear + " constants at line " + (constants.substring(0, keydex).replace("[^\n]", "").length() + 1));
//		}
		int endOfLine = constants.indexOf("\n", keydex);
		if(endOfLine == -1)
		{
			endOfLine = constants.length();
		}
		String sValue = constants.substring(keydex, endOfLine).trim();
		return OrangeUtility.cleanDoubleParse(sValue);
	}
	
	/**
	 * Retrieves the constants file for the specified year.
	 * @param constantsin the constants files String to be searched through
	 * @return the constants file for the Parser's year
	 */
	private String retrieveThisYear(String constantsin)
	{
		ArrayList<String> years = new ArrayList<String>();
		
		/*int sep = constantsin.indexOf(CONSTANTSSEPERATOR);
		while(sep != -1)
		{
			years.add(constantsin.substring(0, sep));
			constantsin = constantsin.substring(sep + 1);
			
			sep = constantsin.indexOf(CONSTANTSSEPERATOR);
		}
		years.add(constantsin);
		//Obsolete code, here in case new code doesn't work
		*/
		
		//Replacement code
		for(String sep : constantsin.split(CONSTANTSSEPERATOR))
		{
			years.add(sep);
		}
		
		for(String possYear : years)
		{
			String[] seperate = possYear.split(YEARSEPERATOR);
			if(seperate[0].trim().equals(constantYear))
			{
				return seperate[1];
			}
		}
		System.out.println("USING BACKUP CONSTANTS!!!!!!!!!!!!!!");
		return BACKUPCONSTANTS;
	}
	
	/**
	 * Checks if the script has another line.
	 * @return True if the script has another line.
	 */
	public boolean hasNextLine()
	{
		return !script.equals("");
	}
	
	/**
	 * @return the script
	 */
	public String getScript()
	{
		return script;
	}
	
	/**
	 * @return the constants this parser is using
	 */
	public String getConstants()
	{
		return constants;
	}
	
	/**
	 * Updates the script and constants for this parser.
	 * @param scriptin the new script
	 * @param constantsin the new constants files
	 */
	public void update(String scriptin, String constantsin)
	{
		script = scriptin;
		constants = retrieveThisYear(constantsin);
	}
}
