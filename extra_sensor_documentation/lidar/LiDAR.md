# Overview
The LiDAR takes a 2D scan and stores the data in a `LaserScan` message ([product page](https://www.makerfabs.com/esp32-uwb-ultra-wideband.html), [github page](https://github.com/Makerfabs/Makerfabs-ESP32-UWB)). The message contains the following fields:

* **header**
	* timestamp in the header is the acquisition time of the first ray in the scan; in frame frame_id, angles are measured around the positive Z axis (counterclockwise, if Z is up) with zero angle being forward along the x axis
* **angle_min**
	* start angle of the scan [radians]
* **angle_max**
	* end angle of the scan [radians]
* **angle_increment**
	* angular distance between measurements [radians]
* **time_increment**
	* time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points float32 scan_time # time between scans [seconds] float32 range_min # minimum range value [m]
* **scan_time**
	* time between scans [seconds]
* **range_min**
	* minimum range value [m]
* **range_max**
	* maximum range value [m]
* **ranges**
	* range data [m] (Note: values < range_min or > range_max should be discarded)
* **intensities**
	* intensity data [device-specific units]. If your device does not provide intensities, please leave the array empty.

# How it works
The `angle_min` and `angle_max` fields specify the field of view for the LiDAR. These values correspond to radians on the unit circle:
![image of a unit circle](<Unit Circle Img.png>)

One of the most important components is the `ranges` array. They contains values for each laser beam shooting out from the LiDAR. We can use the provided fields of `range_min`, `range_max`, and `angle_increment` to figure out which index in the ranges array corresponds to which degree in the car's field of vision:

Assume the following values in radians:
``` python
range_min = -2.35
range_max = 2.35
angle_increment = 0.0043694
```

We are told that the distance between each laser is 0.0043694 radians
We can use this value to calculate the total number of slices (in radians) for the entire field of view (1,080 in this case).
``` python
num_slices = (range_max * 2) / angle_increment
```

Next, we can conver the `angle_increment` from radians to degrees. Remember that Degrees = Radians * 180/pi. In our case, the increment in degrees is 0.25.
```python
deg_increment = total_deg / num_slices
```

Now we know that each index in the array corresponds to 0.25 degrees. We can then finally calculate the index in the ranges array that corresponds to any degree value (between 0 and 270 in our case).
```python
ranges_index = desired_deg_value / deg_increment
```

Here is an example python function to calculate the ranges_index for any degree value:
```python
def get_ranges_index_for_degree(range_min, range_max, angle_increment, desired_deg_value):

    num_slices = (range_max * 2) / angle_increment
    total_deg = (range_max * 2) * (180/pi)
	deg_increment =  total_deg / num_slices
	ranges_index = desired_deg_value / deg_increment
	return ranges_index
```

## Translating to the F1-TENTH Car

Initially, this is the orientation of the car:

![car's intial pose and orientation](<Car Initial Pose.png>)

Notice that straight ahead corresponds to 135° (2.35619 radians) and the x-direction. Directly left corresponds to the negative y-direction and 45°, while directly right corresponds to the positive y-direction and 225°. 

Since our car's LiDAR has a 270° field of view and the length between each laser is 0.25°, that means that there are a total of 270 / 0.25 = 1,080 readings returned by the `LaserScan` message in the `ranges` array. With this knowledge, we can divide the `ranges` array into practical sections:

* ranges\[180]
	* Corresponds to 45° in the diagram above
	* 90° left
* ranges\[900]
	* Corresponds to 225° in the diagram above
	* 90° right
* ranges\[180:-180]
	* Corresponds to 45° to 225° in the diagram above
	* Front view only (180°)
* ranges\[270:540]
	* Corresponds to 60° to 135° in the diagram above
	* Looking ahead left
* ranges\[540:810]
	* Corresponds to 135° to 210° in the diagram above
	* Looking ahead right