// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.FRC5010;

import java.util.ArrayList;

import javax.transaction.xa.Xid;

/** Add your docs here. */
public class LinearInterpolation {

    private ArrayList<Double> xList;
    private ArrayList<Double> yList;

    public LinearInterpolation(){
      xList = new ArrayList<Double>();
      yList = new ArrayList<Double>();
    }

    public void add(double x, double y){
      xList.add(x);
      yList.add(y);
    }
    public double aproximateForY(double xInput) {
        // Linear Interpolation Calculations
        double yAprox;
        double largestBoundVal = xList.get(xList.size() - 1);
        double smallestBoundVal = xList.get(0);

        if(xList.size() < 1){
          if(xInput > largestBoundVal){
            yAprox = largestBoundVal;
          }else if(xInput < smallestBoundVal){
            yAprox = smallestBoundVal;
          }else if(xList.contains(xInput)){
            yAprox = yList.get((int) xInput);
          }else{
            double xUpperBound = Math.ceil(xInput);
            double xLowerBound = Math.floor(xInput);

            for (int i = 0; i < xList.size(); i++) {
              xUpperBound = xList.get(i);
              if(xList.get(i) > xInput)
                break;
            }

            for (int i = xList.size() - 1; i > 0; i--) {
              xLowerBound = xList.get(i);
              if(xList.get(i) < xInput)
                break;
            }
            
    

            double yUpperBound = (double) yList.get((int) xUpperBound);
            double yLowerBound = (double) yList.get((int) xLowerBound);
    
        
            yAprox = (xInput - xLowerBound) * (yUpperBound - yLowerBound) / (xUpperBound - xLowerBound) + yLowerBound;
          }
          
        }else{
          yAprox = 0;
        }

        return yAprox;
      }
    
}
