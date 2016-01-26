package com.roee.opencv.opencv;

/**
 * Created by Roee on 24/12/2015.
 */
public class LinearEquation {
    private double a;
    private double b;

    public LinearEquation(double a, double b){
        this.a = a;
        this.b = b;
    }

    public double getA() {
        return a;
    }

    public void setA(double a) {
        this.a = a;
    }

    public double getB() {
        return b;
    }

    public void setB(double b) {
        this.b = b;
    }

    public Point intersect(LinearEquation linearEquation){
        double x = (getB() - linearEquation.getB()) / (linearEquation.getA() - getA()),
                y = getA() * x + getB();
        return new Point(x, y);
    }

}