package org.usfirst.frc.team3476.ScriptableAuto;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.usfirst.frc.team3476.Utility.OrangeUtility;

/**
 * Parses a scriptable autonomous workingScript with a constants file.
 * @author Anthony Demetrescu
 *
 */
public class Parser
{
	final String BACKUPCONSTANTS = "//Driving deadzones\nDRIVEDEAD = 2\nDRIVESTRAIGHTDEAD = 5\nTURNDEAD = 5\n\n//encoders\nUSELEFT = 1\nUSERIGHT = 1\n\n//Drivestraight PID\nSTRAIGHTP = 1.5\nSTRAIGHTI = 0\nSTRAIGHTD = 0.001\n\n//Drive PID\nDRIVEP = 1.3\nDRIVEI = 0.005\nDRIVED = 0\n\n//Turn PID\nTURNP = 1.7\nTURNI = 0.005\nTURND = 0\n\n//Intake constants\nSUCKMOTORSPEED = -1\nLOADMOTORSPEED = -1\nFORWARDISDOWN = 0 //false\nAIMUPPOWERED = 1 //true\n\n//Shooter constants\nSHOOTEROUTPUTRANGEHIGH = 1\nSHOOTEROUTPUTRANGELOW = -1\nSHOOTERIGAIN = 0.00001\nFLY1DIR = -1\nFLY2DIR = 1\nFLY3DIR = -1\nFLY4DIR = 1\nGRABFRISBEETIME = 0.65\nSHOOTFRISBEETIME = 0.33\nFLYWHEELDEAD = 100\nFLYWHEELMAXSPEED = 3000";
	
	final String PARALLELSEPARATOR = ";", CONSTANTSSEPERATOR = "\u001B", YEARSEPERATOR = "~", FIRSTPARAM = ":", SECONDPARAM = "@", THEN = ">";
	private String workingScript, script, constants, constantYear;
	private ScriptAssembler assembler;

	private String[] scripts;

	private String[] scriptNames;

	private String selected;
	
	/**
	 * Constructor that takes a script, constants files String, and a year identifier String.
	 * @param scriptin the autonomous workingScript String
	 * @param constantsin the constants file String
	 * @param thisYear the year identifier String
	 */
	public Parser(String scriptin, String constantsin, String thisYear)
	{
		assembler = new ScriptAssembler();
		constantYear = thisYear;
		constants = "";
		script = "";
		this.scripts = new String[]{};
		this.scriptNames = new String[]{};
		this.selected = "";
		update(scriptin, constantsin);
	}
	
	public Parser(String scripts[], String scriptNames[], String selected, String constantsin, String thisYear)
	{
		assembler = new ScriptAssembler();
		constantYear = thisYear;
		constants = "";
		script = "";
		this.scripts = new String[]{};
		this.scriptNames = new String[]{};
		this.selected = "";
		update(scripts, scriptNames, selected, constantsin);
	}
	
	public void resetScript()
	{
		System.out.println("resetScript");
		workingScript = script;
	}
	
	/**
	 * Returns the next line of the autonomous workingScript.
	 * @return the ArrayList of CommandBlocks that represent this line.
	 */
	public ArrayList<CommandBlock> nextLine()
	{
		if(workingScript.equals(""))
		{
			return new ArrayList<CommandBlock>();
		}
		int endOfLine = workingScript.indexOf("\n");
		if(endOfLine == -1)
		{
			endOfLine = workingScript.length();
		}
		String line = workingScript.substring(0, endOfLine);//Get the next line
		if (workingScript.length() == endOfLine)
		{
			workingScript = "";//Remove the line we just retrieved
		}
		else
		{
			workingScript = workingScript.substring(endOfLine + 1);
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
		Command result = new Command(command, params);
		return result;
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
			throw new IOException("test");
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
		
		double result = OrangeUtility.cleanDoubleParse(sValue);
		
		if(key.equals("FLY1DIR"))
		{
			//System.out.println("FLY1DIR: " + sValue + ", result: " + result);
		}
		
		return result;
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
				return OrangeUtility.removeSLComments(seperate[1], "//");
			}
		}
		System.out.println("USING BACKUP CONSTANTS!!!!!!!!!!!!!!");
		return OrangeUtility.removeSLComments(BACKUPCONSTANTS, "//");
	}
	
	/**
	 * Checks if the workingScript has another line.
	 * @return True if the workingScript has another line.
	 */
	public boolean hasNextLine()
	{
		return !workingScript.equals("");
	}
	
	/**
	 * @return the script
	 */
	public String getScript()
	{
		return script;
	}
	
	public String getSelectedName()
	{
		return selected;
	}
	
	public String getSelectedScript()
	{
		return assembler.getSelectedScript();
	}
	
	/**
	 * @return the workingScript
	 */
	public String getWorkingScript()
	{
		return workingScript;
	}
	
	/**
	 * @return the constants this parser is using
	 */
	public String getConstants()
	{
		return constants;
	}
	
	/**
	 * Updates the workingScript and constants for this parser.
	 * @param scriptin the new workingScript
	 * @param constantsin the new constants files
	 */
	public void update(String scriptin, String constantsin)
	{
		if(!script.equals(scriptin)) System.out.println("Script different");
		script = scriptin;
		workingScript = script;
		String temp = retrieveThisYear(constantsin);
		if(!temp.equals(constants)) System.out.println("Constants different");
		constants = temp;
	}
	
	public void update(String[] scripts, String[] scriptNames, String selected, String constantsin)
	{
		boolean diff = 	!Arrays.equals(this.scripts, scripts) || !Arrays.equals(this.scriptNames, scriptNames)
						|| !this.selected.equals(selected);
		this.scripts = scripts;
		this.scriptNames = scriptNames;
		this.selected = selected;
		if(diff) System.out.println("Script params different");
		
		HashMap<String, String> scriptMap = new HashMap<>();
		for(int i = 0; i < scripts.length; i++)
		{
			scriptMap.put(scriptNames[i], scripts[i]);
		}
		assembler.update(scriptMap, selected);
		script = assembler.getAssembled();
		
		workingScript = script;
		String temp = retrieveThisYear(constantsin);
		if(!temp.equals(constants)) System.out.println("Constants different");
		constants = temp;
	}
}
