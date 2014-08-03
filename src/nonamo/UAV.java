package nonamo;

import java.io.File;
import java.util.ArrayList;
import java.util.Random;

import cern.jet.random.Exponential;
import cern.jet.random.engine.MersenneTwister;


public class UAV {
    // UAV status codes
    public static final int UAV_IDLE         = 0;   // waiting for a point cloud
    public static final int UAV_TRANSMITTING = 1;   // transmitting data over the network
    public static final int UAV_BACKOFF      = 2;   // waiting for network availability
    
    // constant labels
    private static final int BYTES_PER_RAW_POINT = 2100;    // CITATION: per-pulse size derived from Cho, 2006 figure 18     
    //private static final int BYTES_PER_XYZ_POINT = 24;      // 3 dimensions * 8-byte double-precision float
    private static final int BACKOFF_CAP         = 10;      // CITATION: IEEE 802.3 CSMA/CD standard
    
    // class-wide parameters
    private static int numUAVs = 0;                 // running tally of declared UAVs
    private static String ptCloudDir;               // folder containing source point cloud files
    private static Random unifRnd;                  // uniform random number generator used to select point clouds
    private static Exponential expRnd;              // exponential random number generator used to select detection times
    private static ArrayList<String> ptClouds;      // the list of paths to point clouds
    private static int timesteps = 0;               // time horizon for data generation
    
    // UAV-specific fields
    private double detectRate;                      // lambda parameter for exponentially-distributed detection times
    private int id;                                 // UAV unique identifier
    
    private int detectTime;                         // the time for the next detection
    private int backoffTime;                        // the time for the next transmission attempt after backing off
    private int backoffCount;                       // the number of back-offs so far
    private PointCloud lastDetect;                  // point cloud to retransmit on the next attempt
    private int status;                             // the UAV's current activity status, where this takes on the status flag values above
    
    /**
     * 
     * @param detectRate    lambda parameter for exponentially-distributed detection times
     * @param ptCloudPath   folder containing source point cloud files
     * @param timesteps     time horizon for data generation
     */
    public UAV(double detectRate, String ptCloudPath, int timesteps){
        // save the fields
        this.detectRate = detectRate;
        UAV.ptCloudDir  = (UAV.ptCloudDir == null) ? ptCloudPath : UAV.ptCloudDir;
        UAV.timesteps   = (UAV.timesteps == 0)     ? timesteps   : UAV.timesteps;
        
        
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
        
        // generate the first detection time and set the status to idle with no backoffs
        makeDetectTime(0);
        status       = UAV_IDLE;
        backoffCount = 0;
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
            if(currentTime <= timesteps && currentTime == detectTime){
                int randomIdx      = unifRnd.nextInt(ptClouds.size());
                String randomCloud = ptClouds.get(randomIdx);
                lastDetect         = new PointCloud(randomCloud, currentTime, BYTES_PER_RAW_POINT, this);
                backoffCount       = 0;
                
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
        
        // otherwise, retry on the next step
        else{
            status = UAV_BACKOFF;
            
            // TODO: experiment with different backoff truncations
            if(backoffCount < BACKOFF_CAP)
                backoffCount++;
            
            backoffTime = currentTime + 1;
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
     * Generate the next detection time, exponentially distributed as per the UAV's detect rate
     * 
     * @param currentTime   current time in the simulation, used as an offset in calculating the next detection time
     */
    private void makeDetectTime(int currentTime){
        detectTime = currentTime + (int)Math.ceil(expRnd.nextDouble());
    }
}
