/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package krunchcv_17;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_imgproc;
import edu.wpi.first.smartdashboard.camera.WPICameraExtension;
import edu.wpi.first.smartdashboard.properties.IntegerProperty;
import edu.wpi.first.smartdashboard.properties.Property;
import edu.wpi.first.smartdashboard.properties.StringProperty;
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.DaisyExtensions;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.tables.ITable;
import edu.wpi.first.wpilibj.tables.ITableListener;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.NoSuchElementException;
import java.util.Scanner;
import javax.imageio.ImageIO;
import javax.swing.JOptionPane;

/**
 *
 * @author Sebastián
 */
public class KrunchCV_17{

    public static final String NAME = "Krunch Target Tracker"; // Name of widget in View->Add list
    private WPIColor alignedColor = new WPIColor(0, 255, 0); // Color of overlay when camera is aligned with goal
    private WPIColor unalignedColor = new WPIColor(255, 0, 0); // Color of overlay when camera is unaligned with goal
    private WPIColor centerPointColor = new WPIColor(255, 255, 0); // Color of polygon center point
    
    // Target ratios
    private static final double VERT_RATIO = 4/32.0; // Width / Height
    private static final double HORIZ_RATIO = 4/23.5; // Height / Width
    
    private static final int SMALLEST_TARGET_AREA = 100;
    
    private static final double kShooterOffsetDeg = 0.0; // Offset for shooter
    private static final double kHorizontalFOVDeg = 47.0; // Horizontal field of view of camera

    private static final double kVerticalFOVDeg = 480.0/640.0*kHorizontalFOVDeg; // Vertical field of view of camera *(FROM 640x480 images)

    
    // Widget Property keys
    private final String teamNumberKey = "Team Number";
    private final String ipKey = "Network Table IP";
    private final String portKey = "Network Table Port";
    private final String tableNameKey = "Table name";
    
    // Widget Properties
//    public final IntegerProperty TEAM_NUMBER_PROPERTY = new IntegerProperty(this, teamNumberKey, 79);
//    public final StringProperty IP_PROPERTY = new StringProperty(this, ipKey, "10.0.79.2");
//    public final IntegerProperty PORT_PROPERTY = new IntegerProperty(this, portKey, 1735);
//    public final StringProperty TABLE_NAME_PROPERTY = new StringProperty(this, tableNameKey); // Not really needed (as of now)
    
    
    // Constants that pertain to HSV threshold value file
    private static final String DEFAULT_CSV_FILENAME = "KrunchCVSettings.txt";
    private static final String s_lineSeparator = System.getProperty("line.separator");
    
    // SmartDashboard Key Values (DOUBLES ONLY)
    Map<String, Object> keyMap;
    
    private final String brightKey = "BRIGHTNESS";
    private final String contrastKey = "CONTRAST";
    private final String hueMinKey = "HUE MIN";
    private final String hueMaxKey = "HUE MAX";
    private final String satMinKey = "SAT MIN";
    private final String satMaxKey = "SAT MAX";
    private final String valMinKey = "VAL MIN";
    private final String valMaxKey = "VAL MAX";
    private final String goalAlignToleranceKey = "G.O.A.T.";
    private final String cameraHeightInchesKey = "Camera Height in Inches"; // Height of camera from ground in inches
    private final String shooterTiltedKey = "shooter tilted"; // Whether or not the shooter is tilted
    private final String cameraPitchDegLowKey = "Camera Pitch Degree Low"; // Low angle camera pitch degree
    private final String cameraPitchDegHighKey = "Camera Pitch Degree High"; // High angle camera pitch degree
    private final String topTargetHeightInchesKey = "Top Target Height Inches"; // Height of the top target
    private final String minWidthRectGoalsKey = "Min Width Rect Goals"; // Min width in pixels that vision will consider a goal
    private final String maxWidthRectGoalsKey = "Max Width Rect Goals"; // Max width in pixels that vision will consider a goal
    
    private static final String saveKey = "save"; // Boolean value
    
    private boolean shooterTilted = false;
    
    private boolean saving = false;

    private boolean m_debugMode = false;
    
    private CanvasFrame cf;

    private NetworkTable netTable = null;
    
    // Store JavaCV temporaries as members to reduce memory management during processing
    private opencv_core.CvSize size = null;
    private WPIContour[] contours;
    private ArrayList<WPIContour> vertRects, horizRects;
    private opencv_imgproc.IplConvKernel morphKernel;
    private opencv_core.IplImage bin; // Container for binary image
    private opencv_core.IplImage hsv;
    private opencv_core.IplImage hue1, hue2, hue3;
    private opencv_core.IplImage sat1, sat2;
    private opencv_core.IplImage val1, val2;
    private WPIPoint linePt1, linePt2, linePt3, linePt4;
    private int horizontalOffsetPixels;

    public KrunchCV_17()
    {
        this(false);
    }

    public KrunchCV_17(boolean debug)
    {
        m_debugMode = debug;
        morphKernel = opencv_imgproc.IplConvKernel.create(3, 3, 1, 1, opencv_imgproc.CV_SHAPE_RECT, null);
        
        cf = new CanvasFrame("Binary");
        
        // Create hashmap to store all values with keys
        // The second parameter is the default value
        keyMap = new HashMap<String, Object>();
        keyMap.put(brightKey, 0.0);
        keyMap.put(contrastKey, 0.0);
        keyMap.put(hueMinKey, 0.0);
        keyMap.put(hueMaxKey, 0.0);
        keyMap.put(satMinKey, 0.0);
        keyMap.put(satMaxKey, 0.0);
        keyMap.put(valMinKey, 0.0);
        keyMap.put(valMaxKey, 0.0);
        keyMap.put(goalAlignToleranceKey, 0.0);
        keyMap.put(cameraHeightInchesKey, 0.0);
        keyMap.put(cameraPitchDegLowKey, 0.0);
        keyMap.put(cameraPitchDegHighKey, 0.0);
        keyMap.put(topTargetHeightInchesKey, 0.0);
        keyMap.put(minWidthRectGoalsKey, 0.0);
        keyMap.put(maxWidthRectGoalsKey, 0.0);
        
        // Update Properties (Setup networktable info)
        this.updateFromProperties();

        
        try 
        {
            // Load Settings from CSV File
            this.loadSettingsFile();
        } 
        catch (Exception ex) 
        {
            handleCSVFileError(ex);
        }
        
        DaisyExtensions.init();
    }
    
    private void loadSettingsFile() throws Exception
    {
        try {
            FileReader fr = new FileReader(DEFAULT_CSV_FILENAME);
            BufferedReader br = new BufferedReader(fr);
            String buffer = "";
            while((buffer = br.readLine()) != null)
            {
                if(buffer != "")
                {
                    // Remove unwanted line separators
                    buffer = buffer.replace(s_lineSeparator, "");
                    String key = "";
                    Object objValue = null;

                    // Get key and value
                    String[] temp = null;
                    if(buffer.contains(",") && !buffer.contains(", "))
                    {
                        temp = buffer.split(",");
                        key = temp[0];
                    }
                    else if(buffer.contains(", "))
                    {
                        temp = buffer.split(", ");
                        key = temp[0];
                    }
                    
                    // Determine the correct type
                    if(temp[1].equals("true") || temp[1].equals("false"))
                    {
                        // Boolean
                        objValue = (temp[1].equals("true")) ? true : false;
                    }
                    else if(temp[1].startsWith("\"") && temp[1].endsWith("\""))
                    {
                        // String
                        objValue = temp[1].substring(1, temp[1].length() - 2); // Take off beginning and end quotes
                    }
                    else if(temp[1].equals(""))
                    {
                        // Empty String
                        objValue = temp[1];
                    }
                    else
                    {
                        // Double
                        objValue = Double.valueOf(temp[1]);
                    }

                    // Change corresponding value
                    keyMap.put(key, objValue);

                    if(objValue.getClass() == Boolean.class)
                    {
                        Robot.getTable().putBoolean(key, (Boolean)objValue);
                    }
                    else if(objValue.getClass() == String.class)
                    {
                        Robot.getTable().putString(key, (String)objValue);
                    }
                    else if(objValue.getClass() == Double.class)
                    {
                        Robot.getTable().putNumber(key, (Double)objValue);
                    }
                }
            }
            fr.close();
            
        } catch (FileNotFoundException ex) {
            try {
                // Create new file and add default values
                FileWriter fw = new FileWriter(DEFAULT_CSV_FILENAME);
                
                // Iterate through all keys to generate new default file
                Iterator i = keyMap.entrySet().iterator();
                while(i.hasNext())
                {
                    Map.Entry<String, Object> entry = (Map.Entry<String, Object>) i.next();
                    
                    // Write defaults depending on the data type
                    if(entry.getValue().getClass() == Boolean.class)
                    {
                        fw.write(entry.getKey() + ", " + Boolean.toString((Boolean)entry.getValue()) + s_lineSeparator);
                    }
                    else if(entry.getValue().getClass() == String.class)
                    {
                        fw.write(entry.getKey() + ", " + "\"" + entry.getValue() + "\"" + s_lineSeparator);
                    }
                    else if(entry.getValue().getClass() == Double.class)
                    {
                        fw.write(entry.getKey() + ", " + Double.toString((Double)entry.getValue()) + s_lineSeparator);
                    }
                    
                }
                
                fw.flush();
                fw.close();
                
                // Set NetworkTable values to default depending on data type
                for(String mapKey : keyMap.keySet())
                {
                    if(keyMap.get(mapKey).getClass() == Boolean.class)
                    {
                        if(keyMap.get(mapKey) != null)
                        {
                            Robot.getTable().putBoolean(mapKey, (Boolean)keyMap.get(mapKey));
                        }
                    }
                    else if(keyMap.get(mapKey).getClass() == String.class)
                    {
                        Robot.getTable().putString(mapKey, (String)keyMap.get(mapKey));
                    }
                    else if(keyMap.get(mapKey).getClass() == Double.class)
                    {
                        Robot.getTable().putNumber(mapKey, (Double)keyMap.get(mapKey));
                    }
                }
                
            } catch (IOException iEx) {
                handleCSVFileError(iEx);
            }
        }
    }
    
    private void saveSettingsFile()
    {
        Thread saveThread;
        saveThread = new Thread(){
            @Override
            public void run(){
                try 
                {
                    // Create new file and add default values
                    FileWriter fw = new FileWriter(DEFAULT_CSV_FILENAME);
                    
                    // Iterate through all keys and write current values to file
                    Iterator i = keyMap.entrySet().iterator();
                    while(i.hasNext())
                    {
                        Map.Entry<String, Object> entry = (Map.Entry<String, Object>) i.next();
                        
                        // Write values depending on the data type
                        if(entry.getValue().getClass() == Boolean.class)
                        {
                            fw.write(entry.getKey() + ", " + Boolean.toString((Boolean)entry.getValue()) + s_lineSeparator);
                        }
                        else if(entry.getValue().getClass() == String.class)
                        {
                            fw.write(entry.getKey() + ", " + "\"" + (String)entry.getValue() + "\"" + s_lineSeparator);
                        }
                        else if(entry.getValue().getClass() == Double.class)
                        {
                            fw.write(entry.getKey() + ", " + Double.toString((Double)entry.getValue()) + s_lineSeparator);
                        }
                    }
                    
                    fw.flush();
                    fw.close();
                } 
                catch (FileNotFoundException ex) 
                {
                    handleCSVFileError(ex);
                }
                catch (IOException iEx)
                {
                    handleCSVFileError(iEx);
                }
                
                // State the save is finished
                Robot.getPreferences().putBoolean(saveKey, false);
                saving = false;
              }
        };
        
        // Start Thread
        saveThread.start();
    }
    
    private void updateLocalSettings()
    {
        // Assign Settings Values from SmartDashboard.
        for(String mapKey : keyMap.keySet())
        {
            keyMap.put(mapKey, Robot.getTable().getNumber(mapKey));
            
            if(keyMap.get(mapKey).getClass() == Boolean.class)
            {
                if(keyMap.get(mapKey) != null)
                {
                    keyMap.put(mapKey, Robot.getTable().getBoolean(mapKey));
                }
            }
            else if(keyMap.get(mapKey).getClass() == String.class)
            {
                keyMap.put(mapKey, Robot.getTable().getString(mapKey));
            }
            else if(keyMap.get(mapKey).getClass() == Double.class)
            {
                keyMap.put(mapKey, Robot.getTable().getNumber(mapKey));
            }
        }
    }
    
    private void updateFromProperties() 
    {
        // PROBABLY THE RIGHT WAY TO DO IT
        // Comment this out to do offsite testing
//        Robot.setHost(IP_PROPERTY.getValue());
//        Robot.setPort(PORT_PROPERTY.getValue());
//        Robot.setTeam(TEAM_NUMBER_PROPERTY.getValue());
//        Robot.getTable().addTableListener(this);
        
        // Setup Network table (MUST BE BEFORE LOAD CSV LOGIC)
//        NetworkTable.setTeam(TEAM_NUMBER_PROPERTY.getValue());
//        NetworkTable.setIPAddress(IP_PROPERTY.getValue());
//        netTable = NetworkTable.getTable(TABLE_NAME_PROPERTY.getValue());
    }
    
    public WPIImage processImage(WPIColorImage rawImage)
    {
        double heading = 0.0;
        
        // Get the current heading of the robot first
        if( !m_debugMode )
        {
            try
            {
                heading = Robot.getTable().getNumber("Heading");
            }
            catch( NoSuchElementException e)
            {
            }
            catch( IllegalArgumentException e )
            {
            }
        }

        // If size hasn't been initialized yet
        if( size == null || size.width() != rawImage.getWidth() || size.height() != rawImage.getHeight() )
        {
            size = opencv_core.cvSize(rawImage.getWidth(),rawImage.getHeight());
            bin = opencv_core.IplImage.create(size, 8, 1); // Binary image container
            hsv = opencv_core.IplImage.create(size, 8, 3); // CvSize, depth, number of channels
            hue1 = opencv_core.IplImage.create(size, 8, 1);
            hue2 = opencv_core.IplImage.create(size, 8, 1);
            hue3 = opencv_core.IplImage.create(size, 8, 1);
            val1 = opencv_core.IplImage.create(size, 8, 1);
            val2 = opencv_core.IplImage.create(size, 8, 1);
            horizontalOffsetPixels =  (int)Math.round(kShooterOffsetDeg*(size.width()/kHorizontalFOVDeg));
            
            // Line points for line that goes down the middle of the image when outputed on the dashboard
            linePt1 = new WPIPoint(size.width()/2+horizontalOffsetPixels,size.height()-1);
            linePt2 = new WPIPoint(size.width()/2+horizontalOffsetPixels,0);
            linePt3 = new WPIPoint(0, size.height()/2);
            linePt4 = new WPIPoint(size.width(), size.height()/2);
        }
        // Get the raw IplImages for OpenCV
        opencv_core.IplImage input = DaisyExtensions.getIplImage(rawImage);

        // Convert to HSV color space
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
        opencv_core.cvSplit(hsv, hue1, sat1, val1, null);       // Init to first IplImage

        // Threshold each component separately
        opencv_imgproc.cvThreshold(hue1, hue2, 40, 255, opencv_imgproc.CV_THRESH_BINARY);   // Lower
        opencv_imgproc.cvThreshold(hue1, hue3, 100, 255, opencv_imgproc.CV_THRESH_BINARY_INV); // Upper
        opencv_imgproc.cvThreshold(val1, val2, 100, 255, opencv_imgproc.CV_THRESH_BINARY);
        
        // Combine the results to obtain our binary image which should for the most
        // part only contain pixels that we care about
        opencv_core.cvAnd(hue2, hue3, bin, null);
        opencv_core.cvAnd(val2, bin, bin, null);
        
        // Uncomment the next two lines to see the raw binary image
//        CanvasFrame cf = new CanvasFrame("binary");
//        cf.setLocation(660, 0);
//        cf.showImage(bin.getBufferedImage());
//        
        // Find contours
        WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
        contours = DaisyExtensions.findConvexContours(binWpi);

        // Process image for rectangular goals
        this.processForHotGoals(rawImage, heading);
//
//        // Draw a crosshair (line down the middle)
//        rawImage.drawLine(linePt1, linePt2, alignedColor, 2);
//        
//        // Draw horizontal line in the middle
//        rawImage.drawLine(linePt3, linePt4, alignedColor, 2);
//
//        DaisyExtensions.releaseMemory();
//
//        //System.gc();
//
//        // Look to see if button was pressed to save settings
//        try{
//            if(Robot.getPreferences().getBoolean(saveKey) && !saving){
//                this.saveSettingsFile();
//                saving = true;
//            }
//        }
//        catch( NoSuchElementException e)
//        {
//        }
//        catch( IllegalArgumentException e )
//        {
//        }
//        
        return rawImage;
    }
    
    private void processForHotGoals(WPIColorImage rawImage, double heading) 
    {
        vertRects = new ArrayList<WPIContour>();
        horizRects = new ArrayList<WPIContour>();
        
        // TODO Add scores
        
        // Iterate through interesting blobs (contours)
        for (WPIContour c : contours)
        {
//            System.out.println("(" + c.getX() + "," + c.getY() + ") W: " + c.getWidth() + ", H: " + c.getHeight());
            double ratioVer = ((double) c.getWidth()) / ((double) c.getHeight());
            double ratioHor = ((double) c.getHeight()) / ((double) c.getWidth());
            if(c.getWidth() * c.getHeight() > SMALLEST_TARGET_AREA) // Filter out small blobs
            {
                if (inToler(ratioVer, VERT_RATIO, .25)){
                    // Add this to vertical rectangles
                    vertRects.add(c);
                    rawImage.drawRect(c.getX(), c.getY(), c.getWidth(), c.getHeight(), WPIColor.YELLOW, 2);
                }
                
                if(inToler(ratioHor, HORIZ_RATIO, .5)){
                    // Add this to horizontal rectangles
                    horizRects.add(c);
                    rawImage.drawRect(c.getX(), c.getY(), c.getWidth(), c.getHeight(), WPIColor.YELLOW, 2);
                }
            }
        }
        
        // Pair up verical and horizontal targets (ones closest to each other) and perform score testing
        for(int i=0; i < vertRects.size(); i++)
        {
            int hIndex = -1;
            
            for(int j=0; j < horizRects.size(); j++)
            {
                WPIContour v = vertRects.get(i);
                WPIContour h = horizRects.get(j);

                // Test to see if the horizontal and vertical targets match up
                double dx = Math.abs(((double)v.getX() + v.getWidth()/2.0) - ((double)h.getX() + h.getWidth()/2.0));
                double dy = Math.abs((v.getY() + v.getHeight()) - h.getY());
                double tapeRatio = (double)v.getWidth() / (double)h.getHeight();
                
                boolean goodLR = dx < h.getWidth() * 1.2; // Tolerance constant
                boolean goodVert = dy < v.getHeight() * 1.4; // Tolerance constant
                boolean goodTape = inToler(tapeRatio, 1.0, 0.4);
                
                // If they do, assign this horizontal target to the vertical target
                if(goodLR && goodVert && goodTape){
                    hIndex = j;
                }
            }
            
            // If there are rects in the image
            if(hIndex > -1){
                System.out.println("PAIRED: (" + vertRects.get(i).getX() + "," + vertRects.get(i).getY() + ") & (" + 
                    horizRects.get(hIndex).getX() + "," + horizRects.get(hIndex).getY() + ")");
                
                int dx = Math.abs(vertRects.get(i).getX() - horizRects.get(hIndex).getX());
                if(dx > horizRects.get(hIndex).getWidth()){
                    System.out.println("and is a LEFT GOAL");
                } else {
                    System.out.println("and is a RIGHT GOAL");
                }
            }
        }
    }

    private boolean inToler(double input, double goal, double tolerance){
        return ((input >= goal - tolerance) && (input <= goal + tolerance));
    }
    
    private double boundAngle0to180DegreesWithDirection(double angle)
    {
        // Find coterminal angle
        while(angle >= 360.0)
        {
            angle -= 360.0;
        }
        while(angle < 0.0)
        {
            angle += 360.0;
        }
        
        // Bound left and right sides of circle to 180 degrees.
        // The -1 indicates that it is left
        if(angle > 180.0)
        {
            angle = (360.0 - angle) * -1;
        }
        
        return angle * -1; // Tweak
        
    }

    private void handleMiscError(Exception e)
    {
        e.printStackTrace();
        JOptionPane.showMessageDialog(null,
            "An error occurred in the program.",
            "Error in program",
            JOptionPane.ERROR_MESSAGE);
    }
    
    private void handleCSVFileError(Exception e)
    {
        e.printStackTrace();
        JOptionPane.showMessageDialog(null,
            "An error occurred when attempting to "
            + "open the CSV file. ",
            "Unable to Open CSV File",
            JOptionPane.ERROR_MESSAGE);
    }
    
       public void valueChanged(ITable itable, String key, Object value, boolean newValue) 
    {
        if(key.equals(shooterTiltedKey)) // Update local tilt value
        {
            shooterTilted = (Boolean)value;
        }
        
        if(!newValue) // Make sure this is set to not pay attention to new values (ERRORS WILL HAPPEN AT loadSettingsFile)
        {
            // If key value was changed concerning this widget
            boolean concernsUs = false;
            for(String mapKey : keyMap.keySet())
            {
                // If key is needed by this program
                if(mapKey.equals(key))
                {
                    concernsUs = true;
                }
            }

            if(concernsUs)
            {
                // Reload the values into the widget
                this.updateLocalSettings();
            }
        }
    }

   /* Property changed occurs when one of the widget's properties is changed by
    * right-clicking the widget and hitting "properties." */
        public void propertyChanged(Property property) 
    {
        if(property.equals(ipKey) || property.equals(portKey) || property.equals(teamNumberKey)
                || property.equals(tableNameKey))
        {
            this.updateFromProperties();
        }
    }
    
    
    
    
    public static void main(String[] args)
    {
        if (args.length == 0)
        {
            System.out.println("Usage: Arguments are paths to image files to test the program on");
            return;
        }

        // Create the widget
        KrunchCV_17 widget = new KrunchCV_17(true);

        long totalTime = 0;
        for (int i = 0; i < args.length; i++)
        {
            // Load the image
            WPIColorImage rawImage = null;
            try
            {
                rawImage = new WPIColorImage(ImageIO.read(new File(args[i%args.length])));
            } catch (IOException e)
            {
                System.err.println("Could not find file! " + args[i%args.length]);
                return;
            }
            
            //shows the raw image before processing to eliminate the possibility
            //that both may be the modified image.
            CanvasFrame original = new CanvasFrame("Raw");
            original.showImage(rawImage.getBufferedImage());

            WPIImage resultImage = null;

            // Process image
            long startTime, endTime;
            startTime = System.nanoTime();
            resultImage = widget.processImage(rawImage);
            endTime = System.nanoTime();

            // Display results
            totalTime += (endTime - startTime);
            double milliseconds = (double) (endTime - startTime) / 1000000.0;
            System.out.format("Processing took %.2f milliseconds%n", milliseconds);
            System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
            
            CanvasFrame result = new CanvasFrame("Result");
            result.setLocation(0, 520);
            result.showImage(resultImage.getBufferedImage());

            System.out.println("Waiting for ENTER to continue to next image or exit...");
            Scanner console = new Scanner(System.in);
            console.nextLine();

            if (original.isVisible())
            {
                original.setVisible(false);
                original.dispose();
            }
            if (result.isVisible())
            {
                result.setVisible(false);
                result.dispose();
            }
        }

        double milliseconds = (double) (totalTime) / 1000000.0 / (args.length);
        System.out.format("AVERAGE:%.2f milliseconds%n", milliseconds);
        System.out.format("(%.2f frames per second)%n", 1000.0 / milliseconds);
        System.exit(0);
    }
    
}
