import requests
import google_streetview.api
import os
import math
import time
import pyspeedtest

GOOGLE_API_KEY = 'AIzaSyCYRoKgibcDQW99JtEtxSbGsDdmGbwaq_U'
key = GOOGLE_API_KEY

outdoor = "outdoor"
radius = 50

def pointsAlongRoad(startPosition, destination):
    key = GOOGLE_API_KEY
    index1 = startPosition
    index2 = destination
    base_url = 'https://roads.googleapis.com/v1/snapToRoads?path='
    r = requests.get(base_url + index1 + '|' + index2 + '&interpolate=true&key=' + key)
    response = r.json()
    l = len(response['snappedPoints'])
    alist = []

    for i in range(l):
        lati = (response['snappedPoints'][i]['location']['latitude'])
        longi = (response['snappedPoints'][i]['location']['longitude'])
        index = str(lati) + ',' + str(longi)
        alist.append(index)

    r2 = requests.get(base_url + str(alist[len(alist)-1]) + '|' + index2 + '&interpolate=true&key=' + key)
    response2 = r2.json()
    l2 = len(response2['snappedPoints'])
    alist2 = []

    for i in range(l2):
        lati = (response2['snappedPoints'][i]['location']['latitude'])
        longi = (response2['snappedPoints'][i]['location']['longitude'])
        index = str(lati) + ',' + str(longi)
        alist2.append(index)
    return (alist + alist2)

def routing(origin, destination):
    key = GOOGLE_API_KEY
    base_url = 'https://maps.googleapis.com/maps/api/directions/json?'
    origin = origin
    destination = destination
    r = requests.get(base_url+'origin='+origin+'&destination='+destination+'&key='+key)
    routes = (r.json()['routes'][0]['legs'][0]['steps'])
    distance = (r.json()['routes'][0]['legs'][0]['steps'][0]['distance']['value'])
    start_lat = (r.json()['routes'][0]['legs'][0]['steps'][0]['start_location']['lat'])
    start_lng = (r.json()['routes'][0]['legs'][0]['steps'][0]['start_location']['lng'])
    end_lat = (r.json()['routes'][0]['legs'][0]['steps'][0]['end_location']['lat'])
    end_lng = (r.json()['routes'][0]['legs'][0]['steps'][0]['end_location']['lng'])
    pic_value = math.floor(distance/5)
    if end_lat > start_lat:
        value_for_lat = (end_lat - start_lat)/pic_value
    else:
        value_for_lat = (start_lat - end_lat)/pic_value
    if end_lng > start_lng:
        value_for_lng = (end_lng - start_lng) / pic_value
    else:
        value_for_lng = (start_lng - end_lng)/pic_value
    for i in range(pic_value):
        lat, long = start_lat + value_for_lat * (i+1), start_lng + value_for_lat * (i+1)
        yield (lat, long)


def get_pic(location, heading, pitch, name):
    params = [{
        'size': '900x900',
        'location': location,
        'heading': heading,  # smaller for left
        'pitch': pitch,  # smaller for head down
        'key': GOOGLE_API_KEY,
        'source': outdoor,
        'radius': radius
    }]
    results = google_streetview.api.results(params)
    try:
        results.download_links('downloads')
    except FileExistsError:
        pass
    if os.path.exists('C:/Users/baoti/4th/downloads/gsv_0.jpg'):
        os.rename(r'C:\Users\baoti\4th\downloads\gsv_0.jpg', r'C:\Users\baoti\4th\downloads' + '\\' + name + '.jpg')


def get_latAndLong(address):
    lat, long = None, None
    key = GOOGLE_API_KEY
    base_url = "https://maps.googleapis.com/maps/api/geocode/json"
    endpoint = '{}?address={}&key={}'.format(base_url, address, key)
    r = requests.get(endpoint)
    if r.status_code not in range(200,299):
        return None, None
    try:
        results = r.json()['results'][0]
        lat = results['geometry']['location']['lat']
        lng = results['geometry']['location']['lng']
    except:
        pass
    return lat, lng


