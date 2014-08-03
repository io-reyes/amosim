package amo;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import com.jmatio.io.MatFileReader;
import com.jmatio.types.MLDouble;

import cern.jet.random.Exponential;
import cern.jet.random.engine.MersenneTwister;


public class UAV {
    // UAV status codes
    public static final int UAV_IDLE         = 0;   // waiting for a point cloud
    public static final int UAV_TRANSMITTING = 1;   // transmitting data over the network
    public static final int UAV_BACKOFF      = 2;   // waiting for network availability  
    public static final int UAV_XYZ          = 10;  // converting raw LADAR to XYZ
    public static final int UAV_SEG          = 11;  // segment out the vehicle points
    public static final int UAV_ATR          = 12;  // compute the most likely object match
    
    // constant labels
    private static final int BACKOFF_CAP   = 10;        // CITATION: IEEE 802.3 CSMA/CD standard
    private static final String PERF_FIELD = "curve";   // the name of the performance estimate field in the MAT file
    private static final int BYTES_PER_DOUBLE = 8;      // number of bytes in a double-precision float
    private static final String SUFFIX_LIKES  = ".mat"; // likelihoods file extension
    private static final String LIKES_TABLE   = "res";  // the field that contains the likelihoods table
    
    // class-wide parameters
    private static int numUAVs = 0;                 // running tally of declared UAVs
    private static String ptCloudDir;               // folder containing source point cloud files
    private static Random unifRnd;                  // uniform random number generator used to select point clouds
    private static Exponential expRnd;              // exponential random number generator used to select detection times
    private static ArrayList<String> ptClouds;      // the list of paths to point clouds
    private static String estimatesPath;            // full path to the MAT file containing a performance estimate called "curve"
    private static double[] perfEstimates;          // computational performance estimates, where perfEstimnates[k] corresponds to the expected accuracy for processing k points
    private static int timesteps = 0;               // time horizon for data generation
    private static int xyzPerSecond = 0;            // the number of XYZ points that can be generated from the raw LADAR returns per second
    private static int segmentedPerSecond = 0;      // the number of points that can be segmented per second
    private static int likelihoodsPerSecond = 0;    // the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
    private static String likesDir;                 // the directory containing computed likelihood files
    private static ArrayList<Integer> library;      // indices of objects in this target library (corresponding to columns in the likelihood files)
    private static ArrayList<String> likelihoods;   // the list of paths to likelihood files
    
    // UAV-specific fields
    private double detectRate;                      // lambda parameter for exponentially-distributed detection times
    private int id;                                 // UAV unique identifier
    
    private int detectTime;                         // the time for the next detection
    private int backoffTime;                        // the time for the next transmission attempt after backing off
    private int backoffCount;                       // the number of back-offs so far
    private PointCloud lastDetect;                  // point cloud to retransmit on the next attempt
    private int lastDetectTime;                     // the time of the previous detection
    private int status;                             // the UAV's current activity status, where this takes on the status flag values above
    private int computeCompletionTime;              // time when the current airborne computation is scheduled to finish
    
    private MarkovChain forecast;                   // latest network latency forecast
    private GroundMeasurements groundStatus;        // latest ground status report
    
    /**
     * 
     * @param detectRate            lambda parameter for exponentially-distributed detection times
     * @param ptCloudPath           folder containing source point cloud files
     * @param estimatesPath         full path to the MAT file containing a performance estimate called "curve"
     * @param timesteps             time horizon for data generation
     * @param xyzPerSecond          the number of XYZ points that can be generated from the raw LADAR returns per second
     * @param segmentedPerSecond    the number of points that can be segmented per second
     * @param likelihoodsPerSecond  the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
     * @param likesDir              the directory containing computed likelihood files
     * @param library               indices of objects in this target library (corresponding to columns in the likelihood files)
     */
    public UAV(double detectRate, String ptCloudPath, String estimatesPath, int timesteps, int xyzPerSecond, int segmentedPerSecond, int likelihoodsPerSecond, String likesDir, ArrayList<Integer> library){     
        // save the fields (only save static fields if not yet already initializes)
        this.detectRate             = detectRate;
        UAV.ptCloudDir              = (UAV.ptCloudDir == null)          ? ptCloudPath           : UAV.ptCloudDir;
        UAV.estimatesPath           = (UAV.estimatesPath == null)       ? estimatesPath         : UAV.estimatesPath;
        UAV.timesteps               = (UAV.timesteps == 0)              ? timesteps             : UAV.timesteps;
        UAV.xyzPerSecond            = (UAV.xyzPerSecond == 0)           ? xyzPerSecond          : UAV.xyzPerSecond;
        UAV.segmentedPerSecond      = (UAV.segmentedPerSecond == 0)     ? segmentedPerSecond    : UAV.segmentedPerSecond;  
        UAV.likelihoodsPerSecond    = (UAV.likelihoodsPerSecond  == 0)  ? likelihoodsPerSecond  : UAV.likelihoodsPerSecond;
        UAV.likesDir                = (UAV.likesDir == null)            ? likesDir              : UAV.likesDir;
        UAV.library                 = (UAV.library == null)             ? library               : UAV.library;
        
        // set a unique ID
        id = numUAVs++;
        
        // initialize the random number generators
        unifRnd = (unifRnd == null) ? new Random() : unifRnd;
        expRnd  = (expRnd  == null) ? new Exponential(1 / this.detectRate, new MersenneTwister(unifRnd.nextInt(Integer.MAX_VALUE))) : expRnd;
        
        // get the list of point clouds if it hasn't already been made
        if(ptClouds == null){
            // open the target directory and initialize the list
            File dir           = new File(ptCloudDir);
            String[] fileList  = dir.list(new VehicleXYZFilter());
            ptClouds           = new ArrayList<String>(fileList.length);
            
            // initialize the entries in the list of point clouds
            for(int n = 0; n < fileList.length; n++)
                ptClouds.add("");
            
            // save the "base" filenames into the list (i.e., "1-NOTA-vehicle.xyz" is saved as "1-NOTA" at the 0th index)
            for(String str : fileList){
                // get the ID number of this point cloud, its base name, and save it into the list at the ptNumber - 1 index
                int ptNumber = Integer.parseInt(str.substring(0, str.indexOf('-')));
                String base  = ptCloudDir + "/" + str.replaceFirst("-vehicle\\.xyz", "");
                ptClouds.set(ptNumber - 1, base);
            }
        }
        
        // get the list of likelihood files if it hasn't already been made
        if(likelihoods == null){
            // open the target directory and initialize the list
            File dir          = new File(likesDir);
            String[] fileList = dir.list(new LikelihoodsFilter());
            likelihoods       = new ArrayList<String>(fileList.length);
            
            // initialize the entries in the list of likelihoods
            for(int n = 0; n < fileList.length; n++)
                likelihoods.add("");
            
            // add the paths to the list
            for(String str : fileList){
                int likeNumber  = Integer.parseInt(str.substring(0, str.indexOf('-')));
                String fullPath = likesDir + "/" + str;
                likelihoods.set(likeNumber - 1, fullPath);
            }
        }
        
        // get the performance estimate if it hasn't already been built
        if(perfEstimates == null){
            // open the target MAT file and save the performance estimate as a matrix
            MatFileReader matRead = null;
            
            try{
                matRead = new MatFileReader(new File(estimatesPath));
            } catch(IOException e){
                e.printStackTrace();
                System.out.println("Could not read performanc estimate file " + estimatesPath);
                System.exit(-1);
            }
            
            MLDouble matEstimates = (MLDouble)matRead.getMLArray(PERF_FIELD);
            
            int[] dims    = matEstimates.getDimensions();
            int numPoints = dims[1];
            
            perfEstimates = new double[numPoints];
            
            for(int n = 0; n < numPoints; n++)
                perfEstimates[n] = matEstimates.getReal(n);
        }
        
        // generate the first detection time and set the status to idle with no backoffs
        makeDetectTime(0);
        status         = UAV_IDLE;
        backoffCount   = 0;
        lastDetectTime = 0;
    }
    
    /**
     * 
     * @param currentTime   current second to simulate
     * @return              randomly-selected point cloud if the UAV is to transmit a point cloud at this time, null otherwise
     */
    public PointCloud step(int currentTime){    
        switch(status){
        // generate a new point cloud if there is a detection at this time
        case UAV_IDLE:
            if(currentTime <= timesteps && currentTime == detectTime){      // only generate data within the time horizon
            	// randomly select a point cloud
                int randomIdx      = unifRnd.nextInt(ptClouds.size());
                String randomCloud = ptClouds.get(randomIdx);
                lastDetect         = new PointCloud(randomCloud, currentTime, this);
                backoffCount       = 0;

                // airborne processing dependent on network and ground inputs
                int networkRate = forecast.getCurrentValue();
                int dataSize    = lastDetect.getScenePoints() * PointCloud.BYTES_PER_RAW_POINT;

                if(forecast == null || groundStatus == null)        		// if there is missing system state information, proceed with transmit-all solution
                    return lastDetect;
                
            	// get user preference
                int desiredLatency     = groundStatus.getLatency();
                double desiredAccuracy = groundStatus.getAccuracy();
                int vehiclePointsToUse = lastDetect.getVehiclePoints();
                int scenePointsToUse   = lastDetect.getScenePoints();
                
                while(perfEstimates[vehiclePointsToUse - 1] > desiredAccuracy)	// find the desired number of segmented points to use, based on user accuracy demands
                    vehiclePointsToUse--;
                
                int groundWait = groundStatus.getServiceTime();					// get the estimated ground service time
                
                // compute transmit-all time
                int transmitAllTime  = expectedNetworkWaitTime(currentTime - lastDetectTime, networkRate, dataSize) +                                       // network wait
                                       (int)Math.ceil((dataSize + 0.0) / networkRate) +                                                                     // network transmit (raw)
                                       groundWait +                                                                                                         // ground wait
                                       groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_RAW, vehiclePointsToUse);                               // ground process (all)
                
                // compute transmit XYZ time
                int convertToXYZTime = (int)Math.ceil((scenePointsToUse + 0.0) / xyzPerSecond); 
                int transmitXYZTime  = convertToXYZTime +                                                                                                   // air process (raw -> XYZ)
                                       expectedNetworkWaitTime(currentTime + convertToXYZTime - lastDetectTime, networkRate, dataSize) +                    // network wait
                                       (int)Math.ceil((scenePointsToUse * PointCloud.BYTES_PER_XYZ_POINT + 0.0) / networkRate) +                            // network transmit (XYZ)
                                       groundWait +                                                                                                         // ground wait
                                       groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_XYZ, vehiclePointsToUse);                               // ground process (segment & ATR)
                
                // compute transmit segmented time
                int convertToSegTime = (int)Math.ceil((scenePointsToUse + 0.0) / segmentedPerSecond);
                int transmitSegTime  = convertToXYZTime +                                                                                                   // air process (raw -> XYZ)
                                       convertToSegTime +                                                                                                   // air process (XYZ -> seg)
                                       expectedNetworkWaitTime(currentTime + convertToXYZTime + convertToSegTime - lastDetectTime, networkRate, dataSize) + // network wait
                                       (int)Math.ceil((vehiclePointsToUse * PointCloud.BYTES_PER_XYZ_POINT + 0.0) / networkRate) +                          // network transmit (XYZ segmented)
                                       groundWait +                                                                                                         // ground wait
                                       groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_SEG, vehiclePointsToUse);                               // ground process (ATR only)
                
                // compute transmit results time
                int classifyTime     = (int)Math.ceil((vehiclePointsToUse * library.size() + 0.0) / likelihoodsPerSecond);
                int transmitATRTime  = convertToXYZTime +                                                                                                   // air process (raw -> XYZ)
                                       convertToSegTime +                                                                                                   // air process (XYZ -> seg)
                                       classifyTime     +                                                                                                   // air process (seg -> ATR)
                                       expectedNetworkWaitTime(currentTime + convertToXYZTime + convertToSegTime + classifyTime - lastDetectTime, networkRate, dataSize) + // network wait
                                       (int)Math.ceil((library.size() * BYTES_PER_DOUBLE + 0.0) / networkRate);                                             // network transmit (likelihood vector)
                
                // carry out the policy with the best time
                int bestTime = Math.min(Math.min(transmitAllTime, transmitXYZTime), Math.min(transmitSegTime, transmitATRTime));
                //bestTime = transmitATRTime;
                
                // try to meet user accuracy requirements, minimize time
                if(groundStatus.isAccuracyPriority() || bestTime >= desiredLatency)
                    lastDetect.setVehiclePoints(vehiclePointsToUse);    
                
                // otherwise, try to meet user latency requirements, maximize accuracy
                else{
                    // for the policy resulting in the best time, find the maximum amount of data to send while staying below the max latency                    
                    if(bestTime == transmitAllTime){
                        int indepOfVehiclePoints = bestTime - groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_RAW, vehiclePointsToUse);
                        while(vehiclePointsToUse < lastDetect.getVehiclePoints() && indepOfVehiclePoints + groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_RAW, vehiclePointsToUse) < desiredLatency)
                            vehiclePointsToUse++;
                    }
                    else if(bestTime == transmitXYZTime){
                        int indepOfVehiclePoints = bestTime - groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_XYZ, vehiclePointsToUse);
                        while(vehiclePointsToUse < lastDetect.getVehiclePoints() && indepOfVehiclePoints + groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_XYZ, vehiclePointsToUse) < desiredLatency)
                            vehiclePointsToUse++;
                    }
                    else if(bestTime == transmitSegTime){
                        int indepOfVehiclePoints = bestTime - (int)Math.ceil((vehiclePointsToUse * PointCloud.BYTES_PER_XYZ_POINT + 0.0) / networkRate) - groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_SEG, vehiclePointsToUse);
                        while(vehiclePointsToUse < lastDetect.getVehiclePoints() && indepOfVehiclePoints + (int)Math.ceil((vehiclePointsToUse * PointCloud.BYTES_PER_XYZ_POINT + 0.0) / networkRate) + groundStatus.getProcessingTime(lastDetect, PointCloud.FORMAT_SEG, vehiclePointsToUse) < desiredLatency)
                            vehiclePointsToUse++;
                    }
                    else if(bestTime == transmitATRTime){
                        int indepOfVehiclePoints = bestTime - classifyTime;
                        while(vehiclePointsToUse < lastDetect.getVehiclePoints() && indepOfVehiclePoints + (int)Math.ceil((vehiclePointsToUse * library.size() + 0.0) / likelihoodsPerSecond) < desiredLatency)
                            vehiclePointsToUse++;
                    }
                    
                    lastDetect.setVehiclePoints(vehiclePointsToUse);
                }
                
                // execute decision
                if(bestTime == transmitXYZTime){        // transmit all XYZ
                    lastDetect.nextFormat();
                    lastDetect.decisionUpdate(PointCloud.FORMAT_XYZ);
                    status                = UAV_XYZ;
                    computeCompletionTime = currentTime + convertToXYZTime; 
                    return null;
                }
                else if(bestTime == transmitSegTime){   // transmit partial segmented 
                    lastDetect.nextFormat(); lastDetect.nextFormat();
                    lastDetect.decisionUpdate(PointCloud.FORMAT_SEG);
                    status                = UAV_SEG;
                    computeCompletionTime = currentTime + convertToXYZTime + convertToSegTime;
                    return null;
                }
                else if(bestTime == transmitATRTime){   // transmit results
                    lastDetect.nextFormat(); lastDetect.nextFormat(); lastDetect.nextFormat();
                    lastDetect.decisionUpdate(PointCloud.FORMAT_ATR);
                    status                = UAV_ATR;
                    computeCompletionTime = currentTime + convertToXYZTime + convertToSegTime + classifyTime;
                    return null;
                }
                
                // transmit all raw
                lastDetect.decisionUpdate(PointCloud.FORMAT_RAW);
                return lastDetect;
            }
            
            break;
            
        // process to a whole-scene XYZ point cloud and transmit
        case UAV_XYZ:
            if(currentTime == computeCompletionTime)
                return lastDetect;
            break;
            
        // process to a segmented vehicle XYZ point cloud and transmit
        case UAV_SEG:
            if(currentTime == computeCompletionTime)
                return lastDetect;
            break;
            
        // fully process and transmit classification results
        case UAV_ATR:
            if(currentTime == computeCompletionTime){
                // read in the likelihoods table
                String likesPath      = likesDir + "/" + lastDetect.getName() + SUFFIX_LIKES;
                MatFileReader matRead = null;
                
                try {
                    matRead = new MatFileReader(new File(likesPath));
                } catch (IOException e) {
                    e.printStackTrace();
                    System.out.println("Error with file " + likesPath);
                    status = UAV_IDLE;          // reset the UAV in case something bad happens
                    makeDetectTime(currentTime);
                    return null;
                }
                
                MLDouble likes = (MLDouble)matRead.getMLArray(LIKES_TABLE);
                
                // build a random access order for the likelihoods table (i.e., essentially shuffle the table)
                int[] dims                = likes.getDimensions();
                ArrayList<Integer> access = new ArrayList<Integer>();
                for(int i = 0; i < dims[0]; i++)
                    access.add(i);
                
                Collections.shuffle(access);
                
                // find max likelihood
                int numPts        = lastDetect.getVehiclePoints(); 
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
                
                // save results and attempt to transmit
                lastDetect.processedUpdate(currentTime, lastDetect.getVehiclePoints(), bestMatch);
                return lastDetect;
            }
            break;
            
        // do nothing; wait for network to change UAV's status when transmission finishes 
        case UAV_TRANSMITTING:
            break;
            
        // present the last point cloud if the backoff period has expired 
        case UAV_BACKOFF:
            if(currentTime == backoffTime)
                return lastDetect;
            break;
        }
        
        return null;
    }
    
    /**
     * Used by the network to respond to transmission attempts from this UAV
     * 
     * @param currentTime         current simulated time
     * @param startedTransmitting true if the transmission has started successfully for this UAV, false otherwise 
     */
    public void networkRespond(int currentTime, boolean startedTransmitting){
        // set to transmitting status if that's the case
        if(startedTransmitting)
            status = UAV_TRANSMITTING;
        
        // otherwise, retry on the next time step
        else{
            status = UAV_BACKOFF;
            
            if(backoffCount < BACKOFF_CAP)
                backoffCount++;
            
            backoffTime = currentTime + 1;// + unifRnd.nextInt((int)Math.pow(2, backoffCount));
        }
    }
    
    /**
     * Used by the network to inform this UAV that its transmission has finished; also sets the time for the next detection
     * 
     * @param currentTime   current simulated time
     */
    public void networkFinish(int currentTime){
        status = UAV_IDLE;
        makeDetectTime(currentTime);
    }
    
    /**
     * 
     * @return  this UAV's unique identifier
     */
    public int getID(){
        return id;
    }
    
    /**
     * 
     * @return  the number of objects in the target library
     */
    public int getLibrarySize(){
        return library.size();
    }
    
    /**
     * Updates the network forecast stored on the UAV
     * 
     * @param forecast latest forecast from the network
     */
    public void saveForecast(MarkovChain forecast){
        this.forecast = forecast;
    }
    
    /**
     * Updates the ground forecast stored on the UAV
     * 
     * @param groundStatus  latest forecast from the ground station
     */
    public void saveGroundMeasurements(GroundMeasurements groundStatus){
        this.groundStatus = groundStatus;
    }
    
    /**
     * Generate the next detection time, exponentially distributed as per the UAV's detect rate
     * 
     * @param currentTime   current time in the simulation, used as an offset in calculating the next detection time
     */
    private void makeDetectTime(int currentTime){
        detectTime = currentTime + (int)Math.ceil(expRnd.nextDouble());
    }
    
    /**
     * Compute the expected amount of time to wait based on the number of other UAVs trying to transmit at this time.
     * 
     * @param numOtherUAVs  the number of other UAVs
     * @param timePerDetect amount of time necessary to transmit one detection
     * @return the expected amount of time to wait based on the number of other UAVs trying to transmit at this time.
     */
    private int expectedNetworkWaitTime(int numOtherUAVs, int timePerDetect){
        if(numOtherUAVs == 0)
            return 0;
        else if(numOtherUAVs == 1)
            return (int)Math.ceil(0.5 * timePerDetect);
        
        return (int)Math.ceil((numOtherUAVs / (numOtherUAVs + 1.0)) * (timePerDetect + expectedNetworkWaitTime(numOtherUAVs - 1, timePerDetect)));
    }
    
    /**
     * 
     * 
     * @param timespan
     * @param networkRate
     * @param dataSize
     * @return
     */
    private int expectedNetworkWaitTime(int timespan, int networkRate, int dataSize){
        // estimate the number of UAVs that generated data during this timespan
        int dataUAVs = 0;
        for(int n = 0; n < numUAVs - 1; n++)
            if(Math.ceil(expRnd.nextDouble()) < timespan)
                dataUAVs++;
        
        // estimate the number of UAVs fully serviced during that time
        int timePerDetect   = Math.max((int)Math.ceil((dataSize + 0.0) / networkRate), 1);	// each detection needs to take at least 1 second to transmit
        int detectsServiced = timespan / timePerDetect;
        dataUAVs            = Math.max(dataUAVs - detectsServiced, 0);
        
        return expectedNetworkWaitTime(dataUAVs, timePerDetect);
    }
}
