package amo;

import java.util.ArrayList;
import java.util.LinkedList;


public class GroundMeasurements {
    private int serviceTime;                    // estimated number of time steps to spend in the ground processing queue
    private ArrayList<Integer> library;         // list of objects in the target library
    private double desiredAccuracy;             // user's desired minimum accuracy (0.0 - 1.0)
    private int desiredLatency;                 // user's desired maximum latency (in seconds)
    private boolean accuracyPriority;           // true if accuracy is deemed to be more important than latency
    
    private int xyzRate;                        // the number of points per second that can be converted from raw LADAR to 3D XYZ
    private int segRate;                        // the number of points per second that can be processed by a segmentation algorithm
    private int atrRate;                        // the number of likelihoods that can be computed per second
    private int numMachines;                    // number of parallel compute elements at the ground station
    
    /**
     * 
     * @param xyzRate       the number of points per second that can be converted from raw LADAR to 3D XYZ
     * @param segRate       the number of points per second that can be processed by a segmentation algorithm
     * @param atrRate       the number of likelihoods that can be computed per second
     * @param numMachines   number of parallel compute elements at the ground station
     */
    public GroundMeasurements(int xyzRate, int segRate, int atrRate, int numMachines){
        // save the fields
        this.xyzRate     = xyzRate;
        this.segRate     = segRate;
        this.atrRate     = atrRate;
        this.numMachines = numMachines;
    }
    
    /**
     * Compute the estimated time spent in the compute queue based on the the current items in the queue.
     * Estimated time is defined as sum(work to be done on each point cloud) / parallel machines
     * 
     * @param queue     the current work queue
     * @return          estimated number of time steps to spend in the queue
     */
    public int setServiceTime(LinkedList<PointCloud> queue){
        serviceTime = 0;
        
        for(PointCloud p : queue){
            switch(p.getFormat()){
            case PointCloud.FORMAT_RAW:
                serviceTime += (int)(Math.ceil(p.getScenePoints() / (xyzRate + 0.0) + p.getScenePoints() / (segRate + 0.0) + (p.getVehiclePoints() * library.size()) / (atrRate + 0.0)));
                break;
                
            case PointCloud.FORMAT_XYZ:
                serviceTime += (int)(Math.ceil(p.getScenePoints() / (segRate + 0.0) + (p.getVehiclePoints() * library.size()) / (atrRate + 0.0)));
                break;
                
            case PointCloud.FORMAT_SEG:
                serviceTime += (int)(Math.ceil((p.getVehiclePoints() * library.size()) / (atrRate + 0.0)));
                break;
            }
        }
        
        serviceTime = (int)Math.ceil((serviceTime + 0.0) / numMachines);
        
        return serviceTime;
    }
    
    /**
     * 
     * @param library list of objects in the target library
     */
    public void setLibrary(ArrayList<Integer> library){
        this.library = library;
    }
    
    /**
     * 
     * @param desiredAccuracy   user's desired minimum accuracy (0.0 - 1.0)
     * @param desiredLatency    user's desired maximum latency (in seconds)
     * @param accuracyPriority  true if accuracy is deemed to be more important than latency
     */
    public void setObjective(double desiredAccuracy, int desiredLatency, boolean accuracyPriority){
        this.desiredAccuracy  = (desiredAccuracy >= 0.0 && desiredAccuracy <= 1.0) ? desiredAccuracy : this.desiredAccuracy;
        this.desiredLatency   = (desiredLatency >= 0)                              ? desiredLatency  : this.desiredLatency;
        this.accuracyPriority = accuracyPriority;
    }
    
    /**
     * 
     * @return  estimated number of time steps to spend in the queue
     */
    public int getServiceTime(){
        return serviceTime;
    }
    
    /**
     * 
     * @param p         point cloud to process
     * @param format    PointCloud.FORMAT_RAW, FORMAT_XYZ, FORMAT_SEG, or FORMAT_ATR
     * @param segToUse  number of segmented points to use, as determined by performance estimates
     * @return          estimated number of time steps to process this point cloud based on its format
     */
    public int getProcessingTime(PointCloud p, int format, int segToUse){
        if(format < PointCloud.FORMAT_RAW || format > PointCloud.FORMAT_ATR)    // return really big number on bad format
            return Integer.MAX_VALUE;
        
        segToUse     = Math.min(segToUse, p.getVehiclePoints());
        int procTime = 0;
        
        switch(format){
        case PointCloud.FORMAT_ATR:     // no further processing required for classified data
            return 0;
        
        case PointCloud.FORMAT_RAW:     // time to convert from raw -> XYZ
            procTime += (int)Math.ceil((p.getScenePoints() + 0.0) / xyzRate);
            
        case PointCloud.FORMAT_XYZ:     // time to convert from XYZ -> segmented
            procTime += (int)Math.ceil((p.getScenePoints() + 0.0) / segRate);
            
        case PointCloud.FORMAT_SEG:     // time to produce likelihoods from segmented points and specified library
            procTime += (int)Math.ceil((segToUse * library.size() + 0.0) / atrRate);
        }
        
        return procTime;
    }
    
    /**
     * 
     * @return  the list of objects in the target library
     */
    public ArrayList<Integer> getLibrary(){
        return library;
    }
    
    /**
     * 
     * @return user's desired minimum accuracy (0.0 - 1.0)
     */
    public double getAccuracy(){
        return desiredAccuracy;
    }
    
    /**
     * 
     * @return user's desired maximum latency (in seconds)
     */
    public int getLatency(){
        return desiredLatency;
    }
    
    /**
     * 
     * @return  true if the user demands the desired accuracy value be met, else priority on the timing value
     */
    public boolean isAccuracyPriority(){
        return accuracyPriority;
    }
}
