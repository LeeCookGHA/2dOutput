package pkg2doutput;

import java.awt.Canvas;
import javax.swing.*;
import java.awt.Dimension;
import java.awt.Color;
import java.awt.Graphics;

public class CanvasArea extends Canvas {
    Dimension minSize = new Dimension(1001, 1001);
    Color backgroundColor = Color.black;
    private double chartScale       =  1d;
    private double chartHorScaleMin = -1d * chartScale;
    private double chartHorScaleMax =  chartScale;
    private double chartVirScaleMin = -1d * chartScale;
    private double chartVirScaleMax =  chartScale;
    private double chartHorCenter =    0d;
    private double chartVirCenter =    0d;

    public void circleAtScaled(double x, double y, Color color, int radius){
        // Invert the Y because the origin is top-left in pixels
        Graphics g = this.getGraphics();
        double x_value = ((x - chartHorScaleMin)/(chartHorScaleMax-chartHorScaleMin))*(double)this.getWidth(); 
        double y_value = (((-1d*y) - chartVirScaleMin)/(chartVirScaleMax-chartVirScaleMin))*(double)this.getHeight();
        g.setColor(color);
        g.fillOval((int)x_value-radius, (int)y_value-radius, (radius*2)+1, (radius*2)+1);
    }
        
    public void circleAtPixels(int x_value, int y_value, Color color, int radius){
        Graphics g = this.getGraphics();
        g.setColor(color);
        g.fillOval((int)x_value-radius, (int)y_value-radius, (radius*2)+1, (radius*2)+1);
    }
    
    public void lineBetweenScaled(double xStart, double yStart, double xEnd, double yEnd, Color color) {
        // Invert the Y because the origin is top-left in pixels
        Graphics g = this.getGraphics();
        double xStart_value = ((xStart - chartHorScaleMin)/(chartHorScaleMax-chartHorScaleMin))*(double)this.getWidth();
        double yStart_value = (((-1d*yStart) - chartVirScaleMin)/(chartVirScaleMax-chartVirScaleMin))*(double)this.getHeight();
        double xEnd_value = ((xEnd - chartHorScaleMin)/(chartHorScaleMax-chartHorScaleMin))*(double)this.getWidth();
        double yEnd_value = (((-1d*yEnd) - chartVirScaleMin)/(chartVirScaleMax-chartVirScaleMin))*(double)this.getHeight();
        g.setColor(color);
        g.drawLine((int)xStart_value, (int)yStart_value, (int)xEnd_value, (int)yEnd_value);
    }

    public void lineBetweenPixels(int xStart_value, int yStart_value, int xEnd_value, int yEnd_value, Color color) {
        Graphics g = this.getGraphics();
        g.setColor(color);
        g.drawLine((int)xStart_value, (int)yStart_value, (int)xEnd_value, (int)yEnd_value);
    }
    
    public void recenterWindow(double xCenter, double yCenter) {
        chartHorCenter = xCenter;
        // Invert the Y because the origin is top-left in pixels
        chartVirCenter = -1d*yCenter;
        calcWindowExtents();
    }
    
    public void clearWindow() {
        Graphics g = this.getGraphics();
        setBackground(backgroundColor);
        g.clearRect(0, 0, this.getWidth(), this.getHeight());;
    }

    public void rescaleWindow(double scale) {
        chartScale =  scale;
        calcWindowExtents();
    }
    
    public double getXfromPix(int xPixel) {
        return ((((double)xPixel/(double)this.getWidth())*2d*chartScale)+chartHorScaleMin);
    }
    
    public double getYfromPix(int yPixel) {
        // Invert the Y because the origin is top-left in pixels
        return (-1d*((((double)yPixel/(double)this.getHeight())*2d*chartScale)+chartVirScaleMin));
    }
    
    private void calcWindowExtents( ) {
        chartHorScaleMin = chartHorCenter - chartScale;
        chartHorScaleMax = chartHorCenter + chartScale;
        chartVirScaleMin = chartVirCenter - chartScale;
        chartVirScaleMax = chartVirCenter + chartScale;
    }

    public CanvasArea(Color color) {
        backgroundColor = color;
        setBackground(color);
    }

    public Dimension getMinimumSize() {
        return minSize;
    }

    public Dimension getPreferredSize() {
        return minSize;
    }
}
