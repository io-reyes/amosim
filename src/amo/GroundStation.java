package amo;

import java.util.ArrayList;
import java.util.LinkedList;


public class GroundStation {
    // ground fields
    private LinkedList<PointCloud> workQueue;           // queue of point clouds to process
    private LinkedList<PointCloud> results;             // list of fully processed point clouds and resulting data
    private LinkedList<GroundProcessor> freeProcessors; // list of free ground processing elements
    private LinkedList<GroundProcessor> busyProcessors; // list of working ground processing elements
    
    private GroundMeasurements groundStatus;            // object containing the state of the ground station
    
    /**
     * Initialize the ground station with the given parameters. Also sets its status to idle.
     * 
     * @param xyzPerSecond          the number of XYZ points that can be generated from the raw LADAR returns per second
     * @param segmentedPerSecond    the number of points that can be segmented per second
     * @param likelihoodsPerSecond  the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
     * @param likesDir              the directory containing computed likelihood files
     * @param library               indices of objects in this target library (corresponding to columns in the likelihood files)
     */
    public GroundStation(int numProcs, int xyzPerSecond, int segmentedPerSecond, int likelihoodsPerSecond, String likesDir, ArrayList<Integer> library){
        // initialize the queues and lists
        workQueue      = new LinkedList<PointCloud>();
        results        = new LinkedList<PointCloud>();
        freeProcessors = new LinkedList<GroundProcessor>();
        busyProcessors = new LinkedList<GroundProcessor>();
        
        for(int n = 0; n < numProcs; n++)
            freeProcessors.add(new GroundProcessor(xyzPerSecond, segmentedPerSecond, likelihoodsPerSecond, likesDir, library));
        
        // initialize the measurement object
        groundStatus = new GroundMeasurements(xyzPerSecond, segmentedPerSecond, likelihoodsPerSecond, numProcs);
        groundStatus.setLibrary(library);
        groundStatus.setServiceTime(workQueue);
    }
    
    /**
     * 
     * @param currentTime   current simulated time
     */
    public void step(int currentTime){
        // check if there are any free processors and any work to be done
        while(workQueue.size() > 0 && freeProcessors.size() > 0){
            // dequeue a point cloud and a ground processor
            PointCloud p      = workQueue.remove();
            GroundProcessor g = freeProcessors.remove();
            
            // attempt to have ground processor g process point cloud p; put back in work queue if fail
            if(!g.process(p, currentTime))
                workQueue.push(p);
            
            busyProcessors.add(g);
        }
        
        // have all busy processors take a step and save any finished results
        for(GroundProcessor g : busyProcessors){
            PointCloud p = g.step(currentTime);
            
            // save the result and free up the processor
            if(p != null)
                results.add(p);
            
            if(g.getStatus() == GroundProcessor.PROC_IDLE)
                freeProcessors.add(g);
        }
        
        // remove all free processors from the busy processor list
        for(GroundProcessor g : freeProcessors)
            busyProcessors.remove(g);
        
        // update the measurement object with a new queue delay estimate
        groundStatus.setServiceTime(workQueue);
}
    
    /**
     * Receive a point cloud from the network. Record the receipt time and enter the point cloud into the processing queue.
     * @param p             point cloud to be processed
     * @param currentTime   current simulated time
     */
    public void receive(PointCloud p, int currentTime){
        p.receiveUpdate(currentTime, workQueue.size());
        
        // add to the work queue if there is still processing to be done on the received data, otherwise add to the results
        if(p.getFormat() < PointCloud.FORMAT_ATR)
            workQueue.add(p);
        else{
            p.dequeueUpdate(currentTime);                                                    // it never enters the queue
            p.processedUpdate(currentTime, p.getVehiclePoints(), p.getClassification());     // it's already processed
            results.add(p);
        }
    }
    
    /**
     * 
     * @return the list of fully processed point clouds and results
     */
    public LinkedList<PointCloud> getResults(){
        return results;
    }
    
    /**
     * 
     * @return the number of point clouds in the queue
     */
    public int getQueueSize(){
        return workQueue.size();
    }
    
    /**
     * 
     * @return object containing the state of the ground station
     */
    public GroundMeasurements getGroundMeasurements(){
        return groundStatus;
    }
}
