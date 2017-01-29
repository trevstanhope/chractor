def maprange(val, a, b):
    """ Lineare Map """
    (a1, a2), (b1, b2) = a, b
    return  b1 + ((val - a1) * (b2 - b1) / (a2 - a1))

def calculate_tire_size(w_tire, aspect_ratio, D_rim):
        """ Calculate tire size (defaults to inches) """
        D_tire = (w_tire * 0.0393701) * (aspect_ratio / 100.0) # mm to inch
        size = 2 * D_tire + D_rim
        return size

def calculate_slip(ground_speed, shaft_rpm, effective_radius=28.0, gear_ratio=1.0, tire_size=30):
        """ Calculate Slip """
        axle_rpm = shaft_rpm / gear_ratio
        wheel_speed = 2 * np.pi * (effective_radius / 1e5) * (60 * axle_rpm)
        slip = wheel_kmh / ground_kmh 
        return slip
