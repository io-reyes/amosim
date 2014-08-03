package amo;

import java.util.ArrayList;
import java.util.LinkedList;


public class Simulation {
    // simulation parameters
    private int timesteps;              // number of seconds to simulate
    private int currentTime;            // current time in the simulator
    
    // UAV parameters
    private LinkedList<UAV> UAVs;       // list of UAVs
    
    // network parameters
    private Network network;            // shared wireless network
    
    // ground station parameters
    private GroundStation groundStation;// ground processing station
    
    /**
     * 
     * @param timesteps the number of 1-second timesteps to simulate
     */
    public Simulation(int timesteps){
        // set the time horizon and current time
        this.timesteps = timesteps;
        currentTime    = 0;
    }
    
    /**
     * 
     * @return  the list of fully-procssed point clouds and associated measurements
     */
    public LinkedList<PointCloud> getResults(){
        return groundStation.getResults();
    }
    
    /**
     * 
     * @return the number of point clouds in the ground station's work queue
     */
    public int getQueueSize(){
        return groundStation.getQueueSize();
    }
    
    /**
     * Initialize the UAVs
     * @param numUAVs               number of UAVs
     * @param detectRate            lambda parameter for exponentially-distributed detection times
     * @param ptCloudPath           path to the folder containing SAIC point cloud files
     * @param estimatesPath         full path to the MAT file containing a performance estimate called "curve"
     * @param xyzPerSecond          the number of XYZ points that can be generated from the raw LADAR returns per second
     * @param segmentedPerSecond    the number of points that can be segmented per second
     * @param likelihoodsPerSecond  the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
     * @param likesDir              the directory containing computed likelihood files
     * @param library               indices of objects in this target library (corresponding to columns in the likelihood files)
     */
    public void initializeUAVs(int numUAVs, double detectRate, String ptCloudPath, String estimatesPath, int xyzPerSecond, int segmentedPerSecond, int likelihoodsPerSecond, String likesDir, ArrayList<Integer> library){
        // ensure that the rates are at least 1 per second
        xyzPerSecond         = (xyzPerSecond         < 1) ? 1 : xyzPerSecond;
        segmentedPerSecond   = (segmentedPerSecond   < 1) ? 1 : segmentedPerSecond;
        likelihoodsPerSecond = (likelihoodsPerSecond < 1) ? 1 : likelihoodsPerSecond;
        
        // initialize the UAVs
        UAVs = new LinkedList<UAV>();
        for(int n = 0; n < numUAVs; n++)
            UAVs.add(new UAV(detectRate, ptCloudPath, estimatesPath, timesteps, xyzPerSecond, segmentedPerSecond, likelihoodsPerSecond, likesDir, library));
    }
    
    /**
     * Initialize the network
     * @param transmitRates     the network's possible transmission rates and transition probabilities
     * @param forecastInterval  number of time steps between latency forecast updates
     * @param forecastLength    number of time steps necessary to broadcast latency forecasts
     */
    public void initializeNetwork(MarkovChain transmitRates, int forecastInterval, int forecastLength){
        network = new Network(transmitRates, forecastInterval, forecastLength);
    }
    
    /**
     * Iniitialize the ground station 
     * @param numProcs              number of independent ground processing elements
     * @param xyzPerSecond          the number of XYZ points that can be generated from the raw LADAR returns per second
     * @param segmentedPerSecond    the number of points that can be segmented per second
     * @param likelihoodsPerSecond  the number of likelihoods that can be computed per second, assuming constant polygon counts across models the target library
     * @param likesDir              the directory containing computed likelihood files
     * @param library               indices of objects in this target library (corresponding to columns in the likelihood files)
     */
    public void initializeGroundStation(int numProcs, int xyzPerSecond, int segmentedPerSecond, int likelihoodsPerSecond, String likesDir, ArrayList<Integer> library){
        groundStation = new GroundStation(numProcs, xyzPerSecond, segmentedPerSecond, likelihoodsPerSecond, likesDir, library);
    }
    
    /**
     * Sets the objective function in terms of desired minimum accuracy and maximum latency
     * 
     * @param desiredAccuracy   user's desired minimum accuracy (0.0 - 1.0)
     * @param desiredLatency    user's desired maximum latency (in seconds)
     * @param accuracyPriority  true if accuracy is deemed to be more important than latency
     */
    public void initializeObjective(double desiredAccuracy, int desiredLatency, boolean accuracyPriority){
        groundStation.getGroundMeasurements().setObjective(desiredAccuracy, desiredLatency, accuracyPriority);
    }
    
    /**
     * Simulate one second in the system
     * 
     * @return  true if the simulation is still running, false otherwise
     */
    public boolean step(){
        // NETWORK: transmit any current data in the network, then signal the source UAV and queuing ground station if it finished
        PointCloud finishedCloud = network.step(currentTime);
        
        if(finishedCloud != null){                                                      // if the network finished transmitting a data set
            finishedCloud.getSrc().networkFinish(currentTime);                          // tell the UAV that it is done transmitting
            groundStation.receive(finishedCloud, currentTime);                          // enqueue at the ground station
        }
        else if(finishedCloud == null && network.getStatus() == Network.NET_FORECAST){   // broadcast ground station status when the network is sending its forecast
            // NOTE: forecast update length must be at least 2 for this to trigger
            for(UAV u : UAVs)                
                u.saveGroundMeasurements(groundStation.getGroundMeasurements());
        }
        
        MarkovChain forecast = network.getForecast();                   // get the latest network latency forecast
        
        // UAV: check UAVs for transmissions attempts and respond to those requests
        int networkStatus                     = network.getStatus();
        LinkedList<PointCloud> toTransmitList = new LinkedList<PointCloud>();
        
        for(UAV u : UAVs){              // get a list of point clouds to be transmitted
            u.saveForecast(forecast);   // save the latest forecast
            PointCloud newCloud = u.step(currentTime);                
            
            if(newCloud != null){
                toTransmitList.add(newCloud);
            }
        }
        
        // respond to the requests; if the network is free, accept a random cloud and retry on the rest
        if(networkStatus == Network.NET_IDLE && toTransmitList.size() > 0){
            PointCloud toTransmit = toTransmitList.remove((int)Math.floor((toTransmitList.size() * Math.random())));
            
            network.startTransmit(toTransmit, currentTime);             // start the transmission
            toTransmit.getSrc().networkRespond(currentTime, true);      // tell the UAV that it is transmitting
        }
        
        for(PointCloud p : toTransmitList)
            p.getSrc().networkRespond(currentTime, false);              // tell rejected UAVs to retry

        
        // GROUND: perform ground processing
        groundStation.step(currentTime);
        
        // terminate the simulation after the time horizon and when all point clouds are processed
        if(currentTime > timesteps && groundStation.getQueueSize() == 0 && toTransmitList.size() == 0)
            return false;
        
        // increment the next time step
        currentTime++;
        
        return true;
    }
}
