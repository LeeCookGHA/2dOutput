package pkg2doutput;

public class Spherical3 {
        private static double CLOSE_ENOUGH_TO_ZERO = 0.000000000000001;
    
	public double r;
	public double az;
	public double el;

	public Spherical3 () {
            this.setFromRads(0, 0, 0);
	}

	public Spherical3 (final Spherical3 inVector) {
		this.set(inVector);
	}

	public String toString () {
		return "(" + az + "," + el + "," + r + ")";
	}
	
	public Spherical3 setFromDegrees (double az, double el, double r) {
		this.az = Math.toRadians(az);
		this.el = Math.toRadians(el);
		this.r = r;
		return this;
	}

        public Spherical3 setFromRads (double az, double el, double r) {
            this.az = az;
            this.el = el;
            this.r = r;
            return this;
	}

	public Spherical3 set (final Spherical3 inVector) {
		return this.setFromRads(inVector.az, inVector.el, inVector.r);
	}

	public Spherical3 setFromVector3 (final Vector3 inV) {
            double range = Math.sqrt((inV.x*inV.x)+(inV.y*inV.y)+(inV.z*inV.z));
            double azimuth = Math.atan(inV.z/inV.x);
            double elevation = Math.asin(inV.y/range);
            return this.setFromRads(azimuth, elevation, range);
	}

	public Spherical3 copy () {
		return new Spherical3(this);
	}

}
