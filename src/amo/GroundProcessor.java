package amo;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;

import com.jmatio.io.MatFileReader;
import com.jmatio.types.MLDouble;


public class GroundProcessor {
    // various labels
    private static final String SUFFIX_LIKES = ".mat";
    private static final String LIKES_TABLE  = "res";
    
    // status codes
    public static final int PROC_IDLE = 0;
    public static final int PROC_XYZ  = 1;
    public static final int PROC_SEG  = 2;
    public static final int PROC_ATR  = 3;
    
    // processing fields
    private static int xyzPerSecond;                // the number of XYZ points that can be generated from the raw LADAR returns per second
    private static int segmentedPerSecond;          // the number of points that can be segmented per second
    private static int likelihoodsPerSecond;        // the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
    private static String likesDir;                 // the directory ocontaining compuated likelihood files
    private static ArrayList<Integer> library;      // list of objects in the object library (indices corresponding to columns in the likelihood files)
    
    private PointCloud currentData;                 // current point cloud in processing
    private int currentToXYZ;                       // number of raw LADAR returns to convert to XYZ
    private int currentToSegment;                   // number of points to segment
    private int currentToClassify;                  // number of likelihoods to compute
    
    private int status;                             // compute element status: idle, raw -> xyz, segmentation, or classification
    
    public GroundProcessor(int xyzPerSecond, int segmentedPerSecond, int likelihoodsPerSecond, String likesDir, ArrayList<Integer> library){
        // save the fields
        GroundProcessor.xyzPerSecond         = xyzPerSecond;
        GroundProcessor.segmentedPerSecond   = segmentedPerSecond;
        GroundProcessor.likelihoodsPerSecond = likelihoodsPerSecond;
        GroundProcessor.likesDir             = likesDir;
        GroundProcessor.library              = library;
        
        // set the status to idle
        status = PROC_IDLE;
    }
    
    /**
     * If there is a point cloud at this processing element, take a step to process it; idle otherwise
     * @param currentTime   current simulated time
     * @return              a fully processed point cloud if finished, null otherwise
     */
    public PointCloud step(int currentTime){
        switch(status){
            // do nothing if the ground station is idle
            case PROC_IDLE:
                break;
                
            // convert raw returns to XYZ
            case PROC_XYZ:
                currentToXYZ += -xyzPerSecond;
                
                if(currentToXYZ <= 0)
                    status = PROC_SEG;
                
                break;
                
            // segment the point cloud
            case PROC_SEG:
                currentToSegment += -segmentedPerSecond;
                
                if(currentToSegment <= 0)
                    status = PROC_ATR;
                
                break;
                
            // run the ATR algorithm
            case PROC_ATR:
                // classify the whole LADAR cloud until it's all done
                currentToClassify += -likelihoodsPerSecond;
                
                if(currentToClassify <= 0){
                    // read in the likelihoods table
                    String likesPath = likesDir + "/" + currentData.getName() + SUFFIX_LIKES;
                    MatFileReader matRead = null;
                    
                    try {
                        matRead = new MatFileReader(new File(likesPath));
                    } catch (IOException e) {
                        e.printStackTrace();
                        System.out.println("Error with file " + likesPath);
                        status = PROC_IDLE;          // reset the ground station in case something bad happens
                        return null;
                    }
                    
                    MLDouble likes = (MLDouble)matRead.getMLArray(LIKES_TABLE);                
                    
                    // build a random access order for the likelihoods table (i.e., essentially shuffle the table)
                    int[] dims                = likes.getDimensions();
                    ArrayList<Integer> access = new ArrayList<Integer>();
                    for(int i = 0; i < dims[0]; i++)
                        access.add(i);
                    
                    Collections.shuffle(access);
                    
                    // find the maximum likelihood (currently assuming the MLDouble is 0-indexed)
                    int numPts        = currentData.getVehiclePoints(); 
                    int maxLikelihood = Integer.MIN_VALUE;
                    int bestMatch     = -1;
                    for(Integer i : library){
                        int obj = i.intValue();
                        int sum = 0;
                        
                        for(int pt = 0; pt < numPts; pt++)
                            sum += likes.getReal(access.get(pt), obj - 1);
                        
                        if(sum > maxLikelihood){
                            maxLikelihood = sum;
                            bestMatch     = obj;
                        }
                    }                
                    
                    // save results, reset ground station to idle, return cloud
                    currentData.processedUpdate(currentTime, numPts, bestMatch);
                    status = PROC_IDLE;
                    
                    return currentData;
                }
        }
        
        return null;
    }
    
    /**
     * Begin processing the specified point cloud 
     * @param p             point cloud to process
     * @param currentTime   current simulated time
     * @return true if processing began successfully
     */
    public boolean process(PointCloud p, int currentTime){
        // do nothing if the processor is busy
        if(status != PROC_IDLE)
            return false;
        
        // save the point cloud and update its dequeue time value
        currentData = p;
        currentData.dequeueUpdate(currentTime);
        
        // determine the amount of work to do at each stage
        currentToXYZ      = currentData.getScenePoints();
        currentToSegment  = currentData.getScenePoints();
        currentToClassify = currentData.getVehiclePoints() * library.size();
        
        // set the status based on the state of the data
        switch(p.getFormat()){
        case PointCloud.FORMAT_RAW:
            status = PROC_XYZ;
            break;
            
        case PointCloud.FORMAT_XYZ:
            status = PROC_SEG;
            break;
            
        case PointCloud.FORMAT_SEG:
            status = PROC_ATR;
            break;
            
        case PointCloud.FORMAT_ATR:
            status = PROC_IDLE;
            break;
        }
        
        return true;
    }
    
    public int getStatus(){
        return status;
    }
}
