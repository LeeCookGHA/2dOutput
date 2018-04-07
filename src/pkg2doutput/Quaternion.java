package pkg2doutput;

public class Quaternion {
        // A value to compare to instead of comparing to 0d
        private static double CLOSE_ENOUGH_TO_ZERO = 0.000000000000001;
        
	public double x;
	public double y;
	public double z;
	public double w;

	public Quaternion () {
		setToIdent();
	}

	public Quaternion (double x, double y, double z, double w) {
		setTo(x, y, z, w);
	}

	public Quaternion (Quaternion quaternion) {
		setTo(quaternion);
	}

	public Quaternion (Vector3 axis, double angle) {
		setTo(axis, angle);
	}

	public Quaternion copy () {
		return new Quaternion(this);
	}

	public String toString () {
		return "[" + x + "|" + y + "|" + z + "|" + w + "]";
	}

	public Vector3 transform (Vector3 v) {
                Quaternion temp1 = copy();
		temp1.invert();
		temp1.mulLeft(v.x, v.y, v.z, 0d);
                temp1.mulLeft(this);
		v.x = temp1.x;
		v.y = temp1.y;
		v.z = temp1.z;
		return v;
	}

	public Quaternion setToIdent () {
		return setTo(0, 0, 0, 1);
	}

        public Quaternion setTo (double inX, double inY, double inZ, double inW) {
		x = inX;
		y = inY;
		z = inZ;
		w = inW;
		return this;
	}

	public Quaternion setTo (Quaternion quaternion) {
		return setTo(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
	}

	public Quaternion setTo (Vector3 axis, double angle) {
		return setFromAxisDeg(axis.x, axis.y, axis.z, angle);
	}

	public Quaternion setFromAxisDeg (final Vector3 axis, final double degrees) {
		return setFromAxisDeg(axis.x, axis.y, axis.z, degrees);
	}

	public Quaternion setFromAxisRad (final Vector3 axis, final double radians) {
		return setFromAxisRad(axis.x, axis.y, axis.z, radians);
	}

	public Quaternion setFromAxisDeg (final double x, final double y, final double z, final double degrees) {
		return setFromAxisRad(x, y, z, Math.toRadians(degrees));
	}

	public Quaternion setFromAxisRad (final double x, final double y, final double z, final double radians) {
		double d = Vector3.length(x, y, z);
		if (d < CLOSE_ENOUGH_TO_ZERO) return setToIdent();
		d = 1d / d;
		double l_ang = radians < 0 ? Math.PI - (-radians % Math.PI) : radians % Math.PI;
		double l_sin = (double)Math.sin(l_ang / 2);
		double l_cos = (double)Math.cos(l_ang / 2);
		return setTo(d * x * l_sin, d * y * l_sin, d * z * l_sin, l_cos).norm();
	}

	public Quaternion setFromAttitude (final double pitch, final double roll) {
            if (0d==pitch && 0d==roll){
                this.setTo(0, 0, -1, 0);
            } else {
                Vector3 downVector = new Vector3(0,0,-1);
                Vector3 attVector = new Vector3(Math.sin(Math.toRadians(pitch)),Math.sin(Math.toRadians(roll)),Math.cos(Math.toRadians(roll))*Math.cos(Math.toRadians(pitch)));
                this.setFromCross(downVector, attVector);
            }
            return this;
        }
        
        
        
	public Quaternion setFromCross (final Vector3 v1, final Vector3 v2) {
		final double dot = clamp(v1.dot(v2), -1d, 1d);
		final double angle = (double)Math.acos(dot);
		return setFromAxisRad(v1.y * v2.z - v1.z * v2.y, v1.z * v2.x - v1.x * v2.z, v1.x * v2.y - v1.y * v2.x, angle);
	}

	public Quaternion setFromCross (final double x1, final double y1, final double z1, final double x2, final double y2, final double z2) {
		final double dot = clamp(Vector3.dot(x1, y1, z1, x2, y2, z2), -1d, 1d);
		final double angle = (double)Math.acos(dot);
		return setFromAxisRad(y1 * z2 - z1 * y2, z1 * x2 - x1 * z2, x1 * y2 - y1 * x2, angle);
	}

	static public double clamp (double value, double min, double max) {
		if (value < min) return min;
		if (value > max) return max;
		return value;
	}
        
	public Quaternion norm () {
		double magSq = ((x * x) + (y * y) + (z * z) + (w * w));
		if (magSq > CLOSE_ENOUGH_TO_ZERO) {
                    double mag = Math.sqrt(magSq);
                    w /= mag;
                    x /= mag;
                    y /= mag;
                    z /= mag;
		}
		return this;
	}

	public Quaternion invert () {
		x = -x;
		y = -y;
		z = -z;
		return this;
	}

	public Quaternion mul (final Quaternion other) {
		final double newX = ((w * other.x) + (x * other.w) + (y * other.z) - (z * other.y));
		final double newY = ((w * other.y) + (y * other.w) + (z * other.x) - (x * other.z));
		final double newZ = ((w * other.z) + (z * other.w) + (x * other.y) - (y * other.x));
		final double newW = ((w * other.w) - (x * other.x) - (y * other.y) - (z * other.z));
		x = newX;
		y = newY;
		z = newZ;
		w = newW;
		return this;
	}

	public Quaternion mul (final double inX, final double inY, final double inZ, final double inW) {
		final double newX = ((w * inX) + (x * inW) + (y * inZ) - (z * inY));
		final double newY = ((w * inY) + (y * inW) + (z * inX) - (x * inZ));
		final double newZ = ((w * inZ) + (z * inW) + (x * inY) - (y * inX));
		final double newW = ((w * inW) - (x * inX) - (y * inY) - (z * inZ));
		x = newX;
		y = newY;
		z = newZ;
		w = newW;
		return this;
	}

	public Quaternion mulLeft (Quaternion other) {
		final double newX = ((other.w * x) + (other.x * w) + (other.y * z) - (other.z * y));
		final double newY = ((other.w * y) + (other.y * w) + (other.z * x) - (other.x * z));
		final double newZ = ((other.w * z) + (other.z * w) + (other.x * y) - (other.y * x));
		final double newW = ((other.w * w) - (other.x * x) - (other.y * y) - (other.z * z));
		x = newX;
		y = newY;
		z = newZ;
		w = newW;
		return this;
	}

	public Quaternion mulLeft (final double inX, final double inY, final double inZ, final double inW) {
		final double newX = ((inW * x) + (inX * w) + (inY * z) - (inZ * y));
		final double newY = ((inW * y) + (inY * w) + (inZ * x) - (inX * z));
		final double newZ = ((inW * z) + (inZ * w) + (inX * y) - (inY * x));
		final double newW = ((inW * w) - (inX * x) - (inY * y) - (inZ * z));
		x = newX;
		y = newY;
		z = newZ;
		w = newW;
		return this;
	}

	public Quaternion add (Quaternion quaternion) {
		x += quaternion.x;
		y += quaternion.y;
		z += quaternion.z;
		w += quaternion.w;
		return this;
	}

	public Quaternion add (double inX, double inY, double inZ, double inW) {
		x += inX;
		y += inY;
		z += inZ;
		w += inW;
		return this;
	}

	public boolean isIdentity () {
		return isIdentity (CLOSE_ENOUGH_TO_ZERO);
	}

	public boolean isIdentity (final double tolerance) {
		if (Math.abs(x) > tolerance) return false;
		if (Math.abs(y) > tolerance) return false;
		if (Math.abs(z) > tolerance) return false;
		if (Math.abs(w-1d) > tolerance) return false;
		return true;
	}
	
	public boolean equals (Quaternion other) {
		if (Math.abs(x - other.x) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(y - other.y) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(z - other.z) > CLOSE_ENOUGH_TO_ZERO) return false;
		if (Math.abs(w - other.w) > CLOSE_ENOUGH_TO_ZERO) return false;
		return true;
	}

	public double dot (final Quaternion other) {
		return ((x * other.x) + (y * other.y) + (z * other.z) + (w * other.w));
	}

	public double dot (final double inX, final double inY, final double inZ, final double inW) {
		return ((x * inX) + (y * inY) + (z * inZ) + (w * inW));
	}

	public Quaternion mul (double scalar) {
		x *= scalar;
		y *= scalar;
		z *= scalar;
		w *= scalar;
		return this;
	}
}
