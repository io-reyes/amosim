package amo;

import java.io.File;
import java.io.FilenameFilter;


public class VehicleXYZFilter implements FilenameFilter {

    public boolean accept(File arg0, String arg1) {
        return arg1.endsWith("-vehicle.xyz");
    }

}
