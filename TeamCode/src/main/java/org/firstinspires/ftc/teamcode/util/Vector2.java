package org.firstinspires.ftc.teamcode.util;

public class Vector2 {
    public double x,y;
    public Vector2(double x,double y) {
        this.x = x;
        this.y = y;
    }

    public double magnitude(){
        return Math.sqrt(x * x + y * y);
    }
    public void normalize(){
        if(magnitude() == 0) return;
        x /= magnitude();
        y /= magnitude();
    }

    public void multiply(double factor){
        x *= factor;
        y *= factor;
    }

    public void zero(){
        x = 0;
        y = 0;
    }

    public void invert(){
        x = -x;
        y = -y;
    }
}
