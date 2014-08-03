package amo;

import java.io.BufferedInputStream;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;


public class PointCloud{
    // various labels
    private static final String SUFFIX_UNSEGMENTED = ".xyz";
    private static final String SUFFIX_SEGMENTED   = "-vehicle.xyz";

    public static final int BYTES_PER_RAW_POINT = 2100;     // CITATION: per-pulse size derived from Cho, 2006 figure 18     
    public static final int BYTES_PER_XYZ_POINT = 24;       // 3 dimensions * 8-byte double-precision float
    public static final int BYTES_PER_DOUBLE    = 8;        // number of bytes in a double-precision float
    
    public static final int FORMAT_RAW = 0;                 // raw LADAR data format 
    public static final int FORMAT_XYZ = 1;                 // 3D XYZ point cloud data
    public static final int FORMAT_SEG = 2;                 // 3D XYZ point cloud data after vehicle segmentation
    public static final int FORMAT_ATR = 3;                 // likelihoods vector format
    
    private static int numClouds = 0;   // running tally of generated point clouds
    
    // detection info
    private String ptCloudPath;         // the path to the source point cloud files (with "-header.xml", ".xyz", and "-vehicle.xyz" suffix removed)
    private int detectTime;             // the time (in seconds after simulation start) this point cloud was generated
    private UAV src;                    // the source UAV for this point cloud
    
    private int scenePoints;            // number of points in the unsegmented scene point cloud
    private int vehiclePoints;          // number of points in the segmented vehicle point cloud
    private int id;                     // unique identifier
    
    // transmission info
    private int transmitTime;           // the time this point cloud started to be transmitted
    
    // processing info
    private int currentFormat;          // the point cloud's current format
    private int receiveTime;            // the time this point cloud finished transmitting and entered the processing queue
    private int queueSize;              // the number of jobs queued ahead of this one at receive time
    private int dequeueTime;            // the time this point cloud is serviced at the ground station
    private int processedTime;          // the time this point cloud is classified by an ATR algorithm
    private int pointsProcessed;        // the number of vehicle points processed 
    private int classification;         // cloud's class as computed by ATR algorithms
    private int decisionFormat;         // the format that the UAV has decided to transmit this point cloud
    
    /**
     * 
     * @param ptCloudPath       the path to the source point cloud files (with "-header.xml", ".xyz", and "-vehicle.xyz" suffix removed)
     * @param detectTime        the time (in seconds after simulation start) this point cloud was generated
     * @param bytesPerPoint     number of bytes per point (nominally 3 dimensions * 8-byte double-precision float)
     * @param src               the source UAV for this point cloud
     */
    public PointCloud(String ptCloudPath, int detectTime, UAV src){
        // save the fields
        this.ptCloudPath    = ptCloudPath;
        this.detectTime     = detectTime;
        this.src            = src;
        
        // read in the number of points
        scenePoints   = -1;
        vehiclePoints = -1;
        try {
            scenePoints   = count(ptCloudPath + SUFFIX_UNSEGMENTED);
            vehiclePoints = count(ptCloudPath + SUFFIX_SEGMENTED);
        } catch (IOException e) {
            e.printStackTrace();
            System.exit(-1);
        }
        
        // set the identifier and format
        id             = numClouds++;
        currentFormat  = FORMAT_RAW;
        decisionFormat = FORMAT_RAW;
    }
    
    /**
     * Update this point cloud at transmission time
     * @param transmitTime  when the network began transmitting this point cloud
     */
    public void transmitUpdate(int transmitTime){
        this.transmitTime = transmitTime;
    }
    
    /**
     * Update this point cloud at receipt time at the ground station
     * @param receiveTime   when the ground station received this point cloud for processing
     * @param queueSize     the number of jobs queued ahead of this one at receive time
     */
    public void receiveUpdate(int receiveTime, int queueSize){
        this.receiveTime = receiveTime;
        this.queueSize   = queueSize;
    }
    
    /**
     * Update this point cloud when it is removed from the processing queue
     * @param dequeueTime   when the ground station begins processing this point cloud   
     */
    public void dequeueUpdate(int dequeueTime){
        this.dequeueTime = dequeueTime;
    }
    
    /**
     * Update this point cloud when it is finally classified by an ATR algorithm
     * @param processedTime     time at which the processing finished
     * @param pointsProcessed   number of vehicle points processed
     * @param classification    ID# of the object matched to this point cloud
     */
    public void processedUpdate(int processedTime, int pointsProcessed, int classification){
        this.processedTime   = processedTime;
        this.pointsProcessed = pointsProcessed;
        this.classification  = classification;
    }
    
    /**
     * 
     * @param transmitFormat    the format that the UAV has decided to transmit this point cloud: PointCloud.FORMAT_RAW, FORMAT_XYZ, FORMAT_SEG, FORMAT_ATR
     */
    public void decisionUpdate(int transmitFormat){
        if(transmitFormat >= FORMAT_RAW && transmitFormat <= FORMAT_ATR)
            decisionFormat = transmitFormat;
    }
    
    /**
     * 
     * @return  the size of LADAR data to transmit in bytes, depending on the current processing state of this point cloud.
     */
    public int getBytes(){
       switch(currentFormat){
       case FORMAT_RAW:
           return scenePoints * BYTES_PER_RAW_POINT;
           
       case FORMAT_XYZ:
           return scenePoints * BYTES_PER_XYZ_POINT;
           
       case FORMAT_SEG:
           return vehiclePoints * BYTES_PER_XYZ_POINT;
           
       case FORMAT_ATR:
           return src.getLibrarySize() * BYTES_PER_DOUBLE;
       }
       
       return scenePoints * BYTES_PER_RAW_POINT;
    }
    
    /**
     * 
     * @return  the number of points in the unsegmented whole-scene point cloud
     */
    public int getScenePoints(){
        return scenePoints;
    }
    
    /**
     * 
     * @return  the number of points processed in classifying this cloud
     */
    public int getPointsProcessed(){
        return pointsProcessed;
    }
    
    /**
     * 
     * @return  the number of points in the vehicle-segmented point cloud
     */
    public int getVehiclePoints(){
        return vehiclePoints;
    }
    
    /**
     * 
     * @return  time when the network began transmitting this point cloud
     */
    public int getTransmitTime(){
        return transmitTime;
    }
    
    /**
     * 
     * @return  time when this point cloud was generated
     */
    public int getDetectTime(){
        return detectTime;
    }
    
    /**
     * 
     * @return  time when this point cloud was received at the ground station
     */
    public int getReceiveTime(){
        return receiveTime;
    }
    
    /**
     * 
     * @return  the numbe of jobs queued ahead of this one at receive time
     */
    public int getQueueSize(){
        return queueSize;
    }
    
    /**
     * 
     * @return  time when this point cloud is dequeued at the ground station for processing
     */
    public int getDequeueTime(){
        return dequeueTime;
    }
    
    /**
     * 
     * @return  time when this point cloud is classified by the ATR algorithms
     */
    public int getProcessedTime(){
        return processedTime;
    }
    
    /**
     * 
     * @return the ID# of the object determined to be the best match for this point cloud
     */
    public int getClassification(){
        return classification;
    }
    
    /**
     * 
     * @return  the name of this point cloud (minus path and extension)
     */
    public String getName(){
        return ptCloudPath.substring(ptCloudPath.lastIndexOf('/') + 1, ptCloudPath.length());
    }
    
    /**
     * 
     * @return  the originating UAV
     */
    public UAV getSrc(){
        return src;
    }
    
    /**
     * 
     * @return this point cloud's current format (i.e., how much processing has been done on it)
     */
    public int getFormat(){
        return currentFormat;
    }
    
    /**
     * 
     * @return the format that the UAV has decided to transmit this point cloud
     */
    public int getDecisionFormat(){
        return decisionFormat;
    }
    
    /**
     * 
     * @return  this detection's unique identifier
     */
    public int getDetectNumber(){
        return id;
    }
    
    /**
     * 
     * @return  this object's number in the SAIC data set
     */
    public int getPointCloudNumber(){
        return Integer.parseInt(getName().substring(0, getName().indexOf('-')));
    }
    
    /**
     * Advances this point cloud's format to the next one in the raw --> XYZ --> segmented --> ATR processing sequence
     */
    public void nextFormat(){
        if(currentFormat < FORMAT_ATR)
            currentFormat++;
    }
    
    /**
     * Sets the number of segmented vehicle points to transmit, as determined through performance estimates.
     * 
     * @param vehiclePoints The number of segmented vehicle points to transmit. If this number exceeds the number
     *                      of points in the segmented point cloud, then trasmit the whole cloud.
     */
    public void setVehiclePoints(int vehiclePoints){
        this.vehiclePoints = (vehiclePoints < this.vehiclePoints) ? vehiclePoints : this.vehiclePoints;
    }
    
    public String toString(){
        return "ID: " + id + " | Src UAV: " + src.getID() + " | Name: " + getName() + " | Processing level: " + decisionFormat + " | Classified as: " + classification;
    }
    
    /**
     * From http://stackoverflow.com/questions/453018/number-of-lines-in-a-file-in-java
     * 
     * @param filename
     * @return
     * @throws IOException
     */
    private int count(String filename) throws IOException {
        InputStream is = new BufferedInputStream(new FileInputStream(filename));
        byte[] c = new byte[1024];
        int count = 0;
        int readChars = 0;
        while ((readChars = is.read(c)) != -1) {
            for (int i = 0; i < readChars; ++i) {
                if (c[i] == '\n')
                    ++count;
            }
        }
        return count;
    }
}
