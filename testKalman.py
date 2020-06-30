import geopy
from geopy.distance import VincentyDistance

# given: lat1, lon1, b = bearing in degrees, d = distance in kilometers

origin = geopy.Point(lat1, lon1)
destination = VincentyDistance(kilometers=d).destination(origin, b)

lat2, lon2 = destination.latitude, destination.longitude
