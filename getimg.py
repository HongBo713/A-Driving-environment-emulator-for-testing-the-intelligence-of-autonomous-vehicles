import requests
import google_streetview.api
import os
import re
import math

GOOGLE_API_KEY = 'AIzaSyCYRoKgibcDQW99JtEtxSbGsDdmGbwaq_U'
key = GOOGLE_API_KEY
outdoor = "outdoor"
radius = 50
#path = "C:/Users/baoti/4th/downloads/"


def get_pic(location, heading, pitch, name):
    params = [{
        'size':'900x900',
        'location':location,
        'heading':heading,       #smaller for left
        'pitch':pitch,          #smaller for head down
        'key':GOOGLE_API_KEY,
        'source': outdoor,
        'radius' : radius
    }]
    results = google_streetview.api.results(params)
    results.download_links('downloads')
    if os.path.exists('C:/Users/baoti/4th/downloads/gsv_0.jpg'):
        os.rename(r'C:\Users\baoti\4th\downloads\gsv_0.jpg',r'C:\Users\baoti\4th\downloads' + '\\' + name + '.jpg' )