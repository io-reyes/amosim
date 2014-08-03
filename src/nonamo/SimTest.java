package nonamo;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;


public class SimTest {

    /**
     * @param args
     */
    public static void main(String[] args) {
        // NON-AMO
        
        // simulation parameters
        int NUM_TIMESTEPS  = 3000;
        int TRIALS         = 1;
        int START          = 0;
        String RESULTS_DIR = "output/nonamo";
        
        // UAV parameters
        final int NUM_UAVS       = 4;
        final double DETECT_RATE = 30.0;
        String CLOUD_DIR         = "data/ptclouds";
        
        // network parameters
        int[] transmitRates = new int[7];
        transmitRates[0]    = 5625000;                                      // CITATION: maximum TCDL rate in bytes = 45 Mbits/s (http://articles.janes.com/articles/Janes-C4I-Systems/Tactical-Common-Data-Link-TCDL-airborne-data-terminal-United-States.html)
        transmitRates[1]    = (int)Math.ceil(transmitRates[0] * (5 / 6.0));// 0x -> (5/6)x of the maximum rate
        transmitRates[2]    = (int)Math.ceil(transmitRates[0] * (4 / 6.0));
        transmitRates[3]    = (int)Math.ceil(transmitRates[0] * (3 / 6.0));
        transmitRates[4]    = (int)Math.ceil(transmitRates[0] * (2 / 6.0));
        transmitRates[5]    = (int)Math.ceil(transmitRates[0] * (1 / 6.0));
        transmitRates[6]    = 0;
        MarkovChain m       = new MarkovChain(transmitRates, 0, true);      // initialize the Markov chain with the above states
        m.setTransition(0, 0, 1.0);                                         // constant max-transmit state      
        m.setTransition(5, 5, 1.0);
        
        // ground parameters
        ArrayList<Integer> library = new ArrayList<Integer>();
        for(int i = 1; i <= 55; i++)                                        // add the military models (#1 - #55) to the reference library
            if(i != 24)                                                     // ignore the MLRS model
                library.add(i);
        final int NUM_MACHINES           = 4;                              // number of independent processing elements 
        final int XYZ_PER_SECOND         = 500;                             // CITATION: Cho, 2006, page 12
        final int SEGMENTED_PER_SECOND   = 500000;                          // Assuming negligible segmentation delay
        final int LIKELIHOODS_PER_SECOND = 7700;                            // CITATION: Experimentally-determined on GPU
        final String LIKES_DIR           = "data/likelihoods";
        
        // Create the output folder if it doesn't already exist
        File resultsDir = new File(RESULTS_DIR);
        if(!resultsDir.exists()) {
        	try {
        		resultsDir.mkdirs();
        	}
        	catch(SecurityException e) {
        		System.out.println(e.getMessage());
        		return;
        	}
        }
        
        // initialize and run the simulation for the specified # of trials
        for(int t = START; t < TRIALS; t++){
            Simulation s = new Simulation(NUM_TIMESTEPS);
            s.initializeUAVs(NUM_UAVS, DETECT_RATE, CLOUD_DIR);
            s.initializeNetwork(m);
            s.initializeGroundStation(NUM_MACHINES, XYZ_PER_SECOND, SEGMENTED_PER_SECOND, LIKELIHOODS_PER_SECOND, LIKES_DIR, library);
            
            // run the sim
            int n = 0;       
            while(s.step()){
                if(n % 100 == 0)
                    System.out.println(t + " - " + n);
                
                n++;
            }
            
            // write the results for each detection to a text file
            // for each row: DetectID, ObjectID, Policy, Detect->classify, Classification
            File outFile      = new File(RESULTS_DIR +"/" + t + ".txt");
            FileWriter writer = null;
            try {
                writer = new FileWriter(outFile, true);
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            for(PointCloud p : s.getResults()){
                int detectID          = p.getDetectNumber();
                int objectID          = p.getPointCloudNumber();
                int policy            = 0;
                int detectToProcessed = p.getProcessedTime() - p.getDetectTime();
                int classification    = p.getClassification();
                
                try {
                    writer.write(detectID + "," + objectID + "," + policy + "," + detectToProcessed + "," + classification + "\n");
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
            try {
                writer.close();
            } catch (IOException e) {
                e.printStackTrace();
            }
            
            System.out.println("Done writing " + t + ".txt");
            s = null;
        }
    }

}
