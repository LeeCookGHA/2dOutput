package pkg2doutput;

public class Vector2 {
        private static double CLOSE_ENOUGH_TO_ZERO = 0.000000000000001;
    
	public double x;
	public double y;

	public Vector2 () {
            this.set(0, 0);
	}

	public Vector2 (double x, double y) {
		this.set(x, y);
	}

	public Vector2 (final Vector2 inVector) {
		this.set(inVector);
	}

	public String toString () {
		return "(" + x + "," + y + ")";
	}
	
	public Vector2 set (double x, double y) {
		this.x = x;
		this.y = y;
		return this;
	}

	public Vector2 set (final Vector2 inVector) {
		return this.set(inVector.x, inVector.y);
	}

	public Vector2 copy () {
		return new Vector2(this);
	}
	
	public Vector2 add (final Vector2 inVector) {
		return this.add(inVector.x, inVector.y);
	}

	public Vector2 add (double x, double y) {
		return this.set(this.x + x, this.y + y);
	}

	public Vector2 sub (final Vector2 a_vec) {
		return this.sub(a_vec.x, a_vec.y);
	}

	public Vector2 sub (double x, double y) {
		return this.set(this.x - x, this.y - y);
	}

	public Vector2 scale (double factor) {
		return this.set(this.x * factor, this.y * factor);
	}
	
	public Vector2 scale (final Vector2 other) {
		return this.set(x * other.x, y * other.y);
	}

	public Vector2 scale (double xScale, double yScale) {
		return this.set(this.x * xScale, this.y * yScale);
	}
	
	public static double length (final double inX, final double inY) {
		return Math.sqrt((inX * inX) + (inY * inY));
	}
	
	public double length () {
		return Math.sqrt((x * x) + (y * y));
	}

	public static double lengthSq (final double inX, final double inY) {
		return ((inX * inX) + (inY * inY));
	}
	
	public double lengthSq () {
		return ((x * x) + (y * y));
	}

	public double distance (final Vector2 inVector) {
		final double a = inVector.x - x;
		final double b = inVector.y - y;
		return Math.sqrt((a * a) + (b * b));
	}

	public double distance (double x, double y) {
		final double a = x - this.x;
		final double b = y - this.y;
		return Math.sqrt((a * a) + (b * b));
	}

	public Vector2 norm () {
		final double lengthSq = this.lengthSq();
		if ((Math.abs(lengthSq) < CLOSE_ENOUGH_TO_ZERO) || (Math.abs(lengthSq-1d) < CLOSE_ENOUGH_TO_ZERO)) return this;
		return this.scale(1d / Math.sqrt(lengthSq));
	}

	public static double dot (double x1, double y1, double x2, double y2) {
		return ((x1 * x2) + (y1 * y2));
	}
	
	public double dot (final Vector2 inVector) {
		return ((x * inVector.x) + (y * inVector.y));
	}

	public double dot (double inX, double inY) {
		return ((x * inX) + (y * inY));
	}

	public boolean isUnit () {
		return isUnitWithTol(CLOSE_ENOUGH_TO_ZERO);
	}
	
	public boolean isUnitWithTol (final double tolerance) {
		return Math.abs(lengthSq() - 1d) < tolerance;
	}
	
	public boolean isZero () {
		return isZeroWithTol (CLOSE_ENOUGH_TO_ZERO);
	}

	public boolean isZeroWithTol (final double tolerance) {
		return lengthSq() < tolerance;
	}
	
	public Vector2 setLength (double inLength) {
		double length = length();
                if((length<CLOSE_ENOUGH_TO_ZERO) || (Math.abs(inLength-length)<CLOSE_ENOUGH_TO_ZERO)) return this;
                scale(inLength/length);
		return this;
	}
	
	public Vector2 limit (double limit) {
		double length = length();
                if (length > limit) {
                    scale(limit/length);
                }
		return this;
	}
	
	public Vector2 clamp (double min, double max) {
		final double lengthSq = lengthSq();
		if (lengthSq < CLOSE_ENOUGH_TO_ZERO) return this;
		double maxSq = max * max;
		if (lengthSq > maxSq) return scale(Math.sqrt(maxSq / lengthSq));
		double minSq = min * min;
		if (lengthSq < minSq) return scale(Math.sqrt(minSq / lengthSq));
		return this;
	}
	
	public boolean equals (Vector2 other) {
		if (this == other) return true;
		if (other == null) return false;
		if (Math.abs(other.x - this.x) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(other.y - this.y) > CLOSE_ENOUGH_TO_ZERO) return false;
		return true;
	}

	public boolean equalsWithTolerance (double x, double y, double tolerance) {
		if (Math.abs(x - this.x) > tolerance) return false;
		if (Math.abs(y - this.y) > tolerance) return false;
		return true;
	}
	
	public Vector2 setZero () {
		this.x = 0;
		this.y = 0;
		return this;
	}
        
	public Vector2 rot90AntiClockwise () {
            double tempX = this.x;
            this.x = this.y * (-1d);
            this.y = tempX;
            return this;
        }
        
	public Vector2 rot90Clockwise () {
            double tempY = this.y;
            this.y = this.x * (-1d);
            this.x = tempY;
            return this;
        }
}
