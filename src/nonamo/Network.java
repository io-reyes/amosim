package nonamo;


public class Network {
    // network status codes
    public static final int NET_IDLE         = 0;   // network is free
    public static final int NET_TRANSMITTING = 1;   // network is currently in the middle of a transmission
    
    // network fields
    private MarkovChain transmitRates;              // Markov chain of various transmission rates
    
    private int status;                             // the network's status
    private PointCloud currentData;                 // current data being transmitted
    private int bytesRemaining;                     // the current number of bytes left to transmit
    
    /**
     * 
     * @param transmitRates markov chain of various transmission rates
     */
    public Network(MarkovChain transmitRates){
        this.transmitRates = transmitRates;
    }
    
    
    /**
     * * Simulate one time step of the network. Determine the next transmission rate and attempt to transmit any current
     * data "in the pipe" at that rate.
     * @param currentTime   current simulated time for this step
     * @return              the transmitted point cloud if it has just finished, null otherwise
     */
    public PointCloud step(int currentTime){
        // get the new transmit rate
        int currentTransmitRate = transmitRates.getNextValue();
        
        if(status == NET_TRANSMITTING){
            // transmit at the current rate
            bytesRemaining += -currentTransmitRate;
            
            // check if the transmission finished; update the data set and set network status accordingly
            if(bytesRemaining <= 0){
                status = NET_IDLE;
                
                return currentData;
            }
        }
        
        return null;
    }
    
    /**
     * Attempt to begin transmitting a point cloud.
     * @param               p point cloud to transmit
     * @param currentTime   current simulated time
     * @return              true if the point cloud has been successfully accepted for transmission
     */
    public boolean startTransmit(PointCloud p, int currentTime){
        // check network status, terminate if busy
        if(status == NET_TRANSMITTING)
            return false;
        
        // record the transmit start time
        p.transmitUpdate(currentTime);
        
        // set the status and current transmitting data
        status          = NET_TRANSMITTING;
        currentData     = p;
        bytesRemaining  = p.getSceneBytes(); 
        
        return true;
    }
    
    /**
     * 
     * @return  the network status; can be Network.NET_IDLE or Network.NET_TRANSMITTING
     */
    public int getStatus(){
        return status;
    }
}
