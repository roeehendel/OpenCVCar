package com.roee.opencv.opencv;

/**
 * Created by Roee on 17/02/2016.
 */
public class Point extends org.opencv.core.Point{

    public double distance(Point p){
        return Math.sqrt(Math.pow(this.x-p.x, 2) + Math.pow(this.y-p.y, 2));
    }

}
