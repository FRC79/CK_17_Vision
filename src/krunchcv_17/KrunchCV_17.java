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
import edu.wpi.first.smartdashboard.robot.Robot;
import edu.wpi.first.wpijavacv.DaisyExtensions;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Scanner;
import javax.imageio.ImageIO;

/**
 *
 * @author Sebastián
 */
public class KrunchCV_17 extends WPICameraExtension{

    public static final String NAME = "Hot Goal Finder"; // Name of widget in View->Add list
    
    // Target ratios
    private static final double VERT_RATIO = 4/32.0; // Width / Height
    private static final double HORIZ_RATIO = 4/23.5; // Height / Width
    
    private static final int SMALLEST_TARGET_AREA = 100;
    
    private static final double kShooterOffsetDeg = 0.0; // Offset for shooter
    private static final double kHorizontalFOVDeg = 67.0; // Horizontal field of view of camera

    private static final double kVerticalFOVDeg = 480.0/640.0*kHorizontalFOVDeg; // Vertical field of view of camera *(FROM 640x480 images)

    private boolean m_debugMode = false;
    
    private CanvasFrame cf;

    // Store JavaCV temporaries as members to reduce memory management during processing
    private opencv_core.CvSize size = null;
    private WPIContour[] contours;
    private ArrayList<WPIContour> vertRects, horizRects;
    private opencv_core.IplImage bin; // Container for binary image
    private opencv_core.IplImage hsv;
    private opencv_core.IplImage hue1, hue2, hue3;
    private opencv_core.IplImage sat1;
    private opencv_core.IplImage val1, val2;
    private int horizontalOffsetPixels;

    public KrunchCV_17()
    {
        this(false);
    }

    public KrunchCV_17(boolean debug)
    {
        m_debugMode = debug;
        
        cf = new CanvasFrame("Binary");
        
        DaisyExtensions.init();
    }
    
    @Override
    public WPIImage processImage(WPIColorImage rawImage)
    {
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
        return processForHotGoals(rawImage);
    }
    
    private WPIColorImage processForHotGoals(WPIColorImage rawImage) 
    {
        vertRects = new ArrayList<WPIContour>();
        horizRects = new ArrayList<WPIContour>();
        
        // Iterate through interesting blobs (contours)
        for (WPIContour c : contours)
        {
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
        
        boolean goalFound = false;
        String goalType = "NONE";
        
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
//                System.out.println("PAIRED: (" + vertRects.get(i).getX() + "," + vertRects.get(i).getY() + ") & (" + 
//                    horizRects.get(hIndex).getX() + "," + horizRects.get(hIndex).getY() + ")");
                goalFound = true;
                
                int dx = Math.abs(vertRects.get(i).getX() - horizRects.get(hIndex).getX());
                if(dx > horizRects.get(hIndex).getWidth()){
                    goalType = "LEFT";
                } else {
                    goalType = "RIGHT";
                }
            }
        }
        
        // Send output back to dashboard
        Robot.getTable().putBoolean("GOAL FOUND", goalFound);
        Robot.getTable().putString("GOAL TYPE", goalType);
        
        return rawImage;
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
