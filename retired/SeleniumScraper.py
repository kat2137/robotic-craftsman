# -*- coding: utf-8 -*-
"""
Created on Sat Jul 18 13:01:02 2020

@author: OHyic
"""
#import selenium drivers
from selenium import webdriver
from selenium.webdriver.chrome.options import Options
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC
from selenium.common.exceptions import NoSuchElementException
from ImageProcessor import ImageProcessor

#import helper libraries
import time
import urllib.request
from urllib.parse import urlparse
import os
import sys
import requests
import io
from PIL import Image
import cv2
import numpy as np

#custom patch libraries
import patch

class SeleniumScraper():
    def __init__(self, webdriver_path, headless):
        #check if chromedriver is updated
        while(True):
            try:
                #try going to www.google.com
                options = Options()
                if(headless):
                    options.add_argument('--headless')
                driver = webdriver.Chrome(options=options)
                driver.set_window_size(1400,1050)
                driver.get("https://www.google.com")
                if driver.find_elements(By.ID, "L2AGLb"):
                    driver.find_element(By.ID, "L2AGLb").click()
                break
            except Exception as e:
                print(e)
            
                #patch chromedriver if not available or outdated
                try:
                    driver
                except NameError:
                    is_patched = patch.download_lastest_chromedriver()
                else:
                    is_patched = patch.download_lastest_chromedriver(driver.capabilities['version'])
                if (not is_patched):
                    exit("[ERR] Please update the chromedriver.exe in the webdriver folder according to your chrome version:https://chromedriver.chromium.org/downloads")

        self.driver = driver