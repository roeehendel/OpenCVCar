package com.roee.opencv.opencv;

import org.opencv.core.Point;

/**
 * Created by Roee on 24/12/2015.
 */
public class LinearEquation {
    public double a;
    public double b;
    public Point point1;
    public Point point2;


    public LinearEquation(double a, double b){
        this.a = a;
        this.b = b;
        point1 = new Point(0, y(0));
        point2 = new Point(DrivingActivity.mFrameHeight, y(DrivingActivity.mFrameHeight));
    }

    public LinearEquation(double x1, double y1, double x2, double y2) {
        createFromPoints(x1, y1, x2, y2);

        this.point1 = new Point(x1, y1);
        this.point2 = new Point(x2, y2);
    }

    public LinearEquation(Point p1, Point p2){
        createFromPoints(p1.x, p1.y, p2.x, p2.y);

        this.point1 = p1;
        this.point2 = p2;
    }

    public LinearEquation(double a, Point p){
        double b = p.y - a * p.x;
        this.a = a;
        this.b = b;
    }

    private void createFromPoints(double x1, double y1, double x2, double y2){
        // Find the linear equation parameters (y = a*x + b)
        double a = (y1 - y2) / (x1 - x2);
        double b = y1 - a * x1;

        this.a = a;
        this.b = b;
    }

    public double y(double x){
        return a * x + b;
    }

    public double distanceFromPoint(Point p){
        double d = (-a * p.x + p.y - b) / Math.sqrt(Math.pow(a ,2) + Math.pow(b ,2));
        return d;
    }

    public Point center(){
        return new Point((point1.x + point2.x) / 2, (point1.y + point2.y) / 2);
    }

    public LinearEquation normal(Point p){
        double a = -1 / this.a;
        double b = -a * p.x + p.y;
        return new LinearEquation(a, b);
    }

    public static Point intersect(LinearEquation line1, LinearEquation line2){
        double x = (line1.b - line2.b) / (line2.a - line1.a),
                y = line1.a * x + line1.b;
        return new Point(x, y);
    }

    public static LinearEquation calculateAngleBisector(LinearEquation line1, LinearEquation line2){
        double a1 = line1.a,
                b1 = line1.b,
                a2 = line2.a,
                b2 = line2.b,
                A1 = -a1,
                A2 = - a2,
                B12 = 1,
                C1 = -b1,
                C2 = -b2,
                R1 = Math.sqrt(A1 * A1 + 1),
                R2 = Math.sqrt(A2 * A2 + 1),
                A = A1 / R1 + A2 / R2,
                B = B12 / R1 + B12 / R2,
                C = C1 / R1 + C2 / R2,
                a = -A/B,
                b = -C/B;

        return new LinearEquation(a, b);
    }

    public double length() {
        return Math.sqrt(Math.pow(point1.x - point2.x, 2) + Math.pow(point1.y - point2.y, 2));
    }
}