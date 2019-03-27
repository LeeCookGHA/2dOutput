package pkg2doutput;

public class Vector3 {
        private static double CLOSE_ENOUGH_TO_ZERO = 0.000000000000001;
    
	public double x;
	public double y;
	public double z;

	public Vector3 () {
            this.set(0, 0, 0);
	}

	public Vector3 (double x, double y, double z) {
		this.set(x, y, z);
	}

	public Vector3 (final Vector3 inVector) {
		this.set(inVector);
	}

	public String toString () {
		return "(" + x + "," + y + "," + z + ")";
	}
	
	public Vector3 set (double x, double y, double z) {
		this.x = x;
		this.y = y;
		this.z = z;
		return this;
	}

	public Vector3 set (final Vector3 inVector) {
		return this.set(inVector.x, inVector.y, inVector.z);
	}

	public Vector3 setFromSpherical (final Spherical3 inSph) {
		double cosEl = Math.cos(inSph.el);
		double sinEl = Math.sin(inSph.el);
		double cosAz = Math.cos(inSph.az);
		double sinAz = Math.sin(inSph.az);
                this.x = inSph.r*cosEl*cosAz;
                this.z = inSph.r*sinAz*cosEl;
                this.y = inSph.r*sinEl;
		return this;
	}

	public Vector3 copy () {
		return new Vector3(this);
	}
	
	public Vector3 add (final Vector3 inVector) {
		return this.add(inVector.x, inVector.y, inVector.z);
	}

	public Vector3 add (double x, double y, double z) {
		return this.set(this.x + x, this.y + y, this.z + z);
	}

	public Vector3 sub (final Vector3 a_vec) {
		return this.sub(a_vec.x, a_vec.y, a_vec.z);
	}

	public Vector3 sub (double x, double y, double z) {
		return this.set(this.x - x, this.y - y, this.z - z);
	}

	public Vector3 scale (double factor) {
		return this.set(this.x * factor, this.y * factor, this.z * factor);
	}
	
	public Vector3 scale (final Vector3 other) {
		return this.set(x * other.x, y * other.y, z * other.z);
	}

	public Vector3 scale (double xScale, double yScale, double zScale) {
		return this.set(this.x * xScale, this.y * yScale, this.z * zScale);
	}
	
	public static double length (final double inX, final double inY, final double inZ) {
		return Math.sqrt((inX * inX) + (inY * inY) + (inZ * inZ));
	}
	
	public double length () {
		return Math.sqrt((x * x) + (y * y) + (z * z));
	}

	public static double lengthSq (final double inX, final double inY, final double inZ) {
		return ((inX * inX) + (inY * inY) + (inZ * inZ));
	}
	
	public double lengthSq () {
		return ((x * x) + (y * y) + (z * z));
	}

	public double distance (final Vector3 inVector) {
		final double a = inVector.x - x;
		final double b = inVector.y - y;
		final double c = inVector.z - z;
		return Math.sqrt(a * a + b * b + c * c);
	}

	public double distance (double x, double y, double z) {
		final double a = x - this.x;
		final double b = y - this.y;
		final double c = z - this.z;
		return Math.sqrt(a * a + b * b + c * c);
	}

	public Vector3 norm () {
		final double lengthSq = this.lengthSq();
		if ((Math.abs(lengthSq) < CLOSE_ENOUGH_TO_ZERO) || (Math.abs(lengthSq-1d) < CLOSE_ENOUGH_TO_ZERO)) return this;
		return this.scale(1d / Math.sqrt(lengthSq));
	}

	public static double dot (double x1, double y1, double z1, double x2, double y2, double z2) {
		return ((x1 * x2) + (y1 * y2) + (z1 * z2));
	}
	
	public double dot (final Vector3 inVector) {
		return ((x * inVector.x) + (y * inVector.y) + (z * inVector.z));
	}

	public double dot (double inX, double inY, double inZ) {
		return ((x * inX) + (y * inY) + (z * inZ));
	}

	public Vector3 cross (final Vector3 inVector) {
		return this.set(y * inVector.z - z * inVector.y, z * inVector.x - x * inVector.z, x * inVector.y - y * inVector.x);
	}

	public Vector3 cross (double inX, double inY, double inZ) {
		return this.set(this.y * inZ - this.z * inY, this.z * inX - this.x * inZ, this.x * inY - this.y * inX);
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
	
	public Vector3 setLength (double inLength) {
		double length = length();
                if((length<CLOSE_ENOUGH_TO_ZERO) || (Math.abs(inLength-length)<CLOSE_ENOUGH_TO_ZERO)) return this;
                scale(inLength/length);
		return this;
	}
	
	public Vector3 limit (double limit) {
		double length = length();
                if (length > limit) {
                    scale(limit/length);
                }
		return this;
	}
	
	public Vector3 clamp (double min, double max) {
		final double lengthSq = lengthSq();
		if (lengthSq < CLOSE_ENOUGH_TO_ZERO) return this;
		double maxSq = max * max;
		if (lengthSq > maxSq) return scale(Math.sqrt(maxSq / lengthSq));
		double minSq = min * min;
		if (lengthSq < minSq) return scale(Math.sqrt(minSq / lengthSq));
		return this;
	}
	
	public boolean equals (Vector3 other) {
		if (this == other) return true;
		if (other == null) return false;
		if (Math.abs(other.x - this.x) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(other.y - this.y) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(other.z - this.z) > CLOSE_ENOUGH_TO_ZERO) return false;
		return true;
	}

	public boolean equalsWithTolerance (double x, double y, double z, double tolerance) {
		if (Math.abs(x - this.x) > tolerance) return false;
		if (Math.abs(y - this.y) > tolerance) return false;
		if (Math.abs(z - this.z) > tolerance) return false;
		return true;
	}
	
	public Vector3 setZero () {
		this.x = 0;
		this.y = 0;
		this.z = 0;
		return this;
	}
}
