package com.swerve.auto;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.Stroke;
import java.awt.geom.AffineTransform;
import java.awt.geom.Ellipse2D;
import java.awt.geom.Line2D;
import java.text.DecimalFormat;
import java.util.LinkedList;

import javax.swing.JFrame;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;

/**
 * This class is a basic plotting class using the Java AWT interface. It has basic features which allow the user to plot multiple graphs on one figure, control axis dimensions, and specify colors.
 *
 * This is by all means not an extensive plotter, but it will help visualize data very quickly and accurately. If a more robust plotting function is required, the user is encouraged to use Excel or Matlab. The purpose of this class is to be easy to use with enought automation to have nice graphs with minimal effort, but give the user control over as much as possible, so they can generate the perfect chart.
 *
 * The plotter also features the ability to capture screen shots directly from the right-click menu, this allows the user to copy and paste plots into reports or other documents rather quickly.
 *
 * This class holds an interface similar to that of Matlab.
 *
 * This class currently only supports scatterd line charts.
 */
public class LinePlot extends JPanel {
	private class XYNode {
		double[] x;
		double[] y;
		boolean lineMarker;
		Color lineColor;
		Color markerColor;

		public XYNode() {
			x = null;
			y = null;

			lineMarker = false;
		}
	}

	private final int yPAD = 60; // Controls how far the X- and Y- axis lines are away from the window edge
	private final int xPAD = 70; // Controls how far the X- and Y- axis lines are away from the window edge

	private double upperXtic;
	private double lowerXtic;
	private double upperYtic;
	private double lowerYtic;
	private boolean xGrid;
	private boolean yGrid;

	private double xMax;
	private double xMin;
	private double yMax;
	private double yMin;

	private int xTicCount;
	private double xTicStepSize;
	private double yTicStepSize;

	private boolean userSetYTic;
	private boolean userSetXTic;

	private String xLabel;
	private String yLabel;
	private String titleLabel;
	protected static int count = 0;

	public JFrame frame;
	JPopupMenu menu = new JPopupMenu("Popup");

	/** Link List to hold all different plots on one graph. */
	private LinkedList<XYNode> link;

	/**
	 * Constructor which Plots only Y-axis data.
	 * 
	 * @param yData Is a array of doubles representing the Y-axis values of the data to be plotted.
	 */
	public LinePlot(double[] yData) {
		this(null, yData, Color.red);
	}

	public LinePlot(double[] yData, Color lineColor, Color marker) {
		this(null, yData, lineColor, marker);
	}

	/**
	 * Constructor which Plots chart based on provided x and y data. X and Y arrays must be of the same length.
	 * 
	 * @param xData Is an array of doubles representing the X-axis values of the data to be plotted.
	 * @param yData Is an array of double representing the Y-axis values of the data to be plotted.
	 */
	public LinePlot(double[] xData, double[] yData) {
		this(xData, yData, Color.red, null);
	}

	/**
	 * Constructor which Plots chart based on provided x and y axis data.
	 * 
	 * @param data Is a 2D array of doubles of size Nx2 or 2xN. The plot assumes X is the first dimension, and y data is the second dimension.
	 */
	public LinePlot(double[][] data) {
		this(getXVector(data), getYVector(data), Color.red, null);
	}

	/**
	 * Constructor which plots charts based on provided x and y axis data in a single two dimensional array.
	 * 
	 * @param data Is a 2D array of doubles of size Nx2 or 2xN. The plot assumes X is the first dimension, and y data is the second dimension.
	 * @param lineColor Is the color the user wishes to be displayed for the line connecting each datapoint.
	 * @param markerColor Is the color the user which to be used for the data point. Make this null if the user wishes to not have datapoint markers.
	 */
	public LinePlot(double[][] data, Color lineColor, Color markerColor) {
		this(getXVector(data), getYVector(data), lineColor, markerColor);
	}

	/**
	 * Constructor which plots charts based on provided x and y axis data provided as separate arrays. The user can also specify the color of the adjoining line. Data markers are not displayed.
	 * 
	 * @param xData Is an array of doubles representing the X-axis values of the data to be plotted.
	 * @param yData Is an array of double representing the Y-axis values of the data to be plotted.
	 * @param lineColor Is the color the user wishes to be displayed for the line connecting each datapoint.
	 */
	public LinePlot(double[] xData, double[] yData, Color lineColor) {
		this(xData, yData, lineColor, null);
	}

	/**
	 * Constructor which plots charts based on provided x and y axis data, provided as separate arrays. The user can also specify the color of the adjoining line and the color of the datapoint maker.
	 * 
	 * @param xData Is an array of doubles representing the X-axis values of the data to be plotted.
	 * @param yData Is an array of double representing the Y-axis values of the data to be plotted.
	 * @param lineColor Is the color the user wishes to be displayed for the line connecting each datapoint.
	 * @param markerColor Is the color the user which to be used for the data point. Make this null if the user wishes to not have datapoint markers.
	 */
	public LinePlot(double[] xData, double[] yData, Color lineColor, Color markerColor) {
		this.xLabel = "X Axis";
		this.yLabel = "Y Axis";
		this.titleLabel = "Title";

		this.upperXtic = -Double.MAX_VALUE;
		this.lowerXtic = Double.MAX_VALUE;
		this.upperYtic = -Double.MAX_VALUE;
		this.lowerYtic = Double.MAX_VALUE;
		this.xTicCount = -Integer.MAX_VALUE;

		this.userSetXTic = false;
		this.userSetYTic = false;

		this.link = new LinkedList<XYNode>();

		addData(xData, yData, lineColor, markerColor);

		count++;
		frame = new JFrame("Figure " + count);
		frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		frame.add(this);
		frame.setSize(720, 720);
		frame.setLocationByPlatform(true);
		frame.setVisible(true);
	}

	/**
	 * Adds a plot to an existing figure.
	 * 
	 * @param yData Is a array of doubles representing the Y-axis values of the data to be plotted.
	 * @param color Is the color the user wishes to be displayed for the line connecting each datapoint
	 */
	public void addData(double[] y, Color lineColor) {
		addData(y, lineColor, null);
	}

	public void addData(double[] y, Color lineColor, Color marker) {
		// Can't add y only data unless all other data is y only data.
		for (XYNode data : link) {
			if (data.x != null) {
				throw new Error("All previous chart series need to have only Y data arrays");
			}
		}

		addData(null, y, lineColor, marker);
	}

	public void addData(double[] x, double[] y, Color lineColor) {
		addData(x, y, lineColor, null);
	}

	public void addData(double[][] data, Color lineColor) {
		addData(getXVector(data), getYVector(data), lineColor, null);
	}

	public void addData(double[][] data, Color lineColor, Color marker) {
		addData(getXVector(data), getYVector(data), lineColor, marker);
	}

	public void addData(double[] x, double[] y, Color lineColor, Color marker) {
		XYNode data = new XYNode();

		// Copy y array into node.
		data.y = new double[y.length];
		data.lineColor = lineColor;

		if (marker == null) {
			data.lineMarker = false;
		} else {
			data.lineMarker = true;
			data.markerColor = marker;
		}

		for (int i = 0; i < y.length; i++) {
			data.y[i] = y[i];
		}

		// If X is not null, copy x
		if (x != null) {
			// Can't add x, and y data unless all other data has x and y data
			for (XYNode d : link) {
				if (d.x == null) {
					throw new Error("All previous chart series need to have both X and Y data arrays");
				}
			}

			if (x.length != y.length) {
				throw new Error("X dimension must match Y dimension");
			}

			data.x = new double[x.length];

			for (int i = 0; i < x.length; i++) {
				data.x[i] = x[i];
			}
		}

		link.add(data);
	}

	public void clearData() {
		frame.repaint(); // Null would be saying that JFrame frame doesn't equal anything, not even a JFrame at that point.
		link.clear();
	}

	/**
	 * Main method which paints the panel and shows the figure.
	 */
	@Override
	protected void paintComponent(Graphics g) {
		super.paintComponent(g);
		Graphics2D g2 = (Graphics2D) g;
		g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

		int w = getWidth();
		int h = getHeight();

		// Draw X and Y lines axis.
		Line2D yaxis = new Line2D.Double(xPAD, yPAD, xPAD, h - yPAD);
		Line2D.Double xaxis = new Line2D.Double(xPAD, h - yPAD, w - xPAD, h - yPAD);
		g2.draw(yaxis);
		g2.draw(xaxis);

		// Find Max Y limits.
		getMinMax(link);

		// Draw ticks.
		drawYTickRange(g2, yaxis, 15, yMax, yMin);
		drawXTickRange(g2, xaxis, 15, xMax, xMin);

		// Plot all data.
		plot(g2);

		// Draw x and y labels.
		setXLabel(g2, xLabel);
		setYLabel(g2, yLabel);
		setTitle(g2, titleLabel);
	}

	public void setXTic(double lowerBound, double upperBound, double stepSize) {
		this.userSetXTic = true;
		this.upperXtic = upperBound;
		this.lowerXtic = lowerBound;
		this.xTicStepSize = stepSize;
	}

	public void setYTic(double lowerBound, double upperBound, double stepSize) {
		this.userSetYTic = true;
		this.upperYtic = upperBound;
		this.lowerYtic = lowerBound;
		this.yTicStepSize = stepSize;
	}

	private void plot(Graphics2D g) {
		int w = super.getWidth();
		int h = super.getHeight();

		Color tempC = g.getColor();

		// Loop through list and plot each.
		for (int i = 0; i < link.size(); i++) {
			// Draw lines.
			double xScale = (w - 2 * xPAD) / (upperXtic - lowerXtic);
			double yScale = (h - 2 * yPAD) / (upperYtic - lowerYtic);

			for (int j = 0; j < link.get(i).y.length - 1; j++) {
				double x1;
				double x2;

				if (link.get(i).x == null) {
					x1 = xPAD + j * xScale;
					x2 = xPAD + (j + 1) * xScale;
				} else {
					x1 = xPAD + xScale * link.get(i).x[j] + lowerXtic * xScale;
					x2 = xPAD + xScale * link.get(i).x[j + 1] + lowerXtic * xScale;
				}

				double y1 = h - yPAD - yScale * link.get(i).y[j] + lowerYtic * yScale;
				double y2 = h - yPAD - yScale * link.get(i).y[j + 1] + lowerYtic * yScale;

				g.setPaint(link.get(i).lineColor);
				g.draw(new Line2D.Double(x1, y1, x2, y2));

				if (link.get(i).lineMarker) {
					g.setPaint(link.get(i).markerColor);
					g.fill(new Ellipse2D.Double(x1 - 2, y1 - 2, 4, 4));
					g.fill(new Ellipse2D.Double(x2 - 2, y2 - 2, 4, 4));
				}
			}
		}

		g.setColor(tempC);
	}

	/**
	 * Need to optimize for loops
	 * 
	 * @param list
	 */
	private void getMinMax(LinkedList<XYNode> list) {
		for (XYNode node : list) {
			double tempYMax = getMax(node.y);
			double tempYMin = getMin(node.y);

			if (tempYMin < yMin) {
				yMin = tempYMin;
			}

			if (tempYMax > yMax) {
				yMax = tempYMax;
			}

			if (xTicCount < node.y.length) {
				xTicCount = node.y.length;
			}

			if (node.x != null) {
				double tempXMax = getMax(node.x);
				double tempXMin = getMin(node.x);

				if (tempXMin < xMin) {
					xMin = tempXMin;
				}

				if (tempXMax > xMax) {
					xMax = tempXMax;
				}
			} else {
				xMax = node.y.length - 1;
				xMin = 0;
			}
		}
	}

	private double getMax(double[] data) {
		double max = -Double.MAX_VALUE;

		for (int i = 0; i < data.length; i++) {
			if (data[i] > max) {
				max = data[i];
			}
		}

		return max;
	}

	private double getMin(double[] data) {
		double min = Double.MAX_VALUE;

		for (int i = 0; i < data.length; i++) {
			if (data[i] < min) {
				min = data[i];
			}
		}

		return min;
	}

	public void setYLabel(String s) {
		yLabel = s;
	}

	public void setXLabel(String s) {
		xLabel = s;
	}

	public void setTitle(String s) {
		titleLabel = s;
	}

	private void setYLabel(Graphics2D g, String s) {
		FontMetrics fm = getFontMetrics(getFont());
		int width = fm.stringWidth(s);

		AffineTransform temp = g.getTransform();

		AffineTransform at = new AffineTransform();
		at.setToRotation(-Math.PI / 2, 10, getHeight() / 2 + width / 2);
		g.setTransform(at);

		// Draw string in center of y axis.
		g.drawString(s, 10, 7 + getHeight() / 2 + width / 2);
		g.setTransform(temp);
	}

	private void setXLabel(Graphics2D g, String s) {
		FontMetrics fm = getFontMetrics(getFont());
		int width = fm.stringWidth(s);

		g.drawString(s, getWidth() / 2 - (width / 2), getHeight() - 10);
	}

	private void setTitle(Graphics2D g, String s) {
		FontMetrics fm = getFontMetrics(getFont());
		String[] line = s.split("\n");

		int height = xPAD / 2 - ((line.length - 1) * fm.getHeight() / 2);

		for (int i = 0; i < line.length; i++) {
			int width = fm.stringWidth(line[i]);
			g.drawString(line[i], getWidth() / 2 - (width / 2), height);
			height += fm.getHeight();
		}
	}

	public void yGridOn() {
		yGrid = true;
		// super.repaint();
	}

	public void yGridOff() {
		yGrid = false;
		// super.repaint();
	}

	public void xGridOn() {
		xGrid = true;
		// super.repaint();
	}

	public void xGridOff() {
		xGrid = false;
		// super.repaint();
	}

	private void drawYTickRange(Graphics2D g, Line2D yaxis, int tickCount, double max, double min) {
		if (!userSetYTic) {
			double range = max - min;

			// Calculate max Y and min Y tic Range.
			double unroundedTickSize = range / (tickCount - 1);
			double x = Math.ceil(Math.log10(unroundedTickSize) - 1);
			double pow10x = Math.pow(10, x);
			yTicStepSize = Math.ceil(unroundedTickSize / pow10x) * pow10x;

			// Determine min and max tick label.
			if (min < 0) {
				lowerYtic = yTicStepSize * Math.floor(min / yTicStepSize);
			} else {
				lowerYtic = yTicStepSize * Math.ceil(min / yTicStepSize);
			}

			if (max < 0) {
				upperYtic = yTicStepSize * Math.floor(1 + max / yTicStepSize);
			} else {
				upperYtic = yTicStepSize * Math.ceil(1 + max / yTicStepSize);
			}
		}

		double x0 = yaxis.getX1();
		double y0 = yaxis.getY1();
		double xf = yaxis.getX2();
		double yf = yaxis.getY2();

		// Calculate stepsize between ticks and length of Y axis using distance formula.
		int roundedTicks = (int) ((upperYtic - lowerYtic) / yTicStepSize);
		double distance = Math.sqrt(Math.pow((xf - x0), 2) + Math.pow((yf - y0), 2)) / roundedTicks;
		double upper = upperYtic;

		for (int i = 0; i <= roundedTicks; i++) {
			double newY = y0;

			// Calculate width of number for proper drawing.
			String number = new DecimalFormat("#.#").format(upper);
			FontMetrics fm = getFontMetrics(getFont());
			int width = fm.stringWidth(number);

			g.draw(new Line2D.Double(x0, newY, x0 - 10, newY));
			g.drawString(number, (float) x0 - 15 - width, (float) newY + 5);

			// Add grid lines to chart.
			if (yGrid && i != roundedTicks) {
				Stroke tempS = g.getStroke();
				Color tempC = g.getColor();

				g.setColor(Color.lightGray);
				g.setStroke(new BasicStroke(1f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1f, new float[] { 5f }, 0f));

				g.draw(new Line2D.Double(xPAD, newY, getWidth() - xPAD, newY));

				g.setColor(tempC);
				g.setStroke(tempS);
			}

			upper = upper - yTicStepSize;
			y0 = newY + distance;
		}
	}

	private void drawXTickRange(Graphics2D g2, Line2D xaxis, int tickCount, double max, double min) {
		drawXTickRange(g2, xaxis, tickCount, max, min, 1);
	}

	private void drawXTickRange(Graphics2D g2, Line2D xaxis, int tickCount, double max, double min, double skip) {
		if (!userSetXTic) {
			double range = max - min;

			// Calculate max Y and min Y tic Range.
			double unroundedTickSize = range / (tickCount - 1);
			double x = Math.ceil(Math.log10(unroundedTickSize) - 1);
			double pow10x = Math.pow(10, x);
			xTicStepSize = Math.ceil(unroundedTickSize / pow10x) * pow10x;

			// Determine min and max tick label.
			if (min < 0) {
				lowerXtic = xTicStepSize * Math.floor(min / xTicStepSize);
			} else {
				lowerXtic = xTicStepSize * Math.ceil(min / xTicStepSize);
			}

			if (max < 0) {
				upperXtic = xTicStepSize * Math.floor(1 + max / xTicStepSize);
			} else {
				upperXtic = xTicStepSize * Math.ceil(1 + max / xTicStepSize);
			}
		}

		double x0 = xaxis.getX1();
		double y0 = xaxis.getY1();
		double xf = xaxis.getX2();
		double yf = xaxis.getY2();

		// Calculate stepsize between ticks and length of Y axis using distance formula.
		int roundedTicks = (int) ((upperXtic - lowerXtic) / xTicStepSize);

		double distance = Math.sqrt(Math.pow((xf - x0), 2) + Math.pow((yf - y0), 2)) / roundedTicks;
		double lower = lowerXtic;

		for (int i = 0; i <= roundedTicks; i++) {
			double newX = x0;

			// Calculate width of number for proper drawing.
			String number = new DecimalFormat("#.#").format(lower);
			FontMetrics fm = getFontMetrics(getFont());
			int width = fm.stringWidth(number);

			g2.draw(new Line2D.Double(newX, yf, newX, yf + 10));

			// Don't label every x tic to prevent clutter.
			if (i % skip == 0)
				g2.drawString(number, (float) (newX - (width / 2.0)), (float) yf + 25);

			// Add grid lines to chart.
			if (xGrid && i != 0) {
				Stroke tempS = g2.getStroke();
				Color tempC = g2.getColor();

				g2.setColor(Color.lightGray);
				g2.setStroke(new BasicStroke(1f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1f, new float[] { 5f }, 0f));

				g2.draw(new Line2D.Double(newX, yPAD, newX, getHeight() - yPAD));

				g2.setColor(tempC);
				g2.setStroke(tempS);
			}

			lower = lower + xTicStepSize;
			x0 = newX + distance;
		}
	}

	public void updateData(int series, double[][] data) {
		// Add Data to link list.
		addData(data, null, null);

		// Copy data from new to old and line styles from list to new list.

		link.get(series).x = link.getLast().x.clone();
		link.get(series).y = link.getLast().y.clone();

		// Remove last data.
		link.removeLast();
	}

	public static double[] getXVector(double[][] array) {
		double[] temp = new double[array.length];

		for (int i = 0; i < temp.length; i++) {
			temp[i] = array[i][0];
		}

		return temp;
	}

	public static double[] getYVector(double[][] array) {
		double[] temp = new double[array.length];

		for (int i = 0; i < temp.length; i++) {
			temp[i] = array[i][1];
		}

		return temp;
	}
}