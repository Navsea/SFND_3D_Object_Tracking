FP.1
implemented matchBoundingBoxes in camFusion_Studen.cpp

tested with cout print outs
_____________
general remarks: 
Im unsure of the efficiency.
Im curious if the solution has more elegant or more efficient method.
_____________
pseudo code:
Step1: 
for every keypoint match
	loop over all the bounding boxes of both the previous and current frame
	see if the keypoint match is contained in the current bounding box
	if it is, add the keypoint to the bounding box struct
	if it is, add the keypoint match to the bounding box struct
	if it is, store the bounding box id and the keypoint match as a multimap
--> this results in a map of bounding box id's and keypoint matches

Step2:
for all the bounding boxes in the previous frame
	get an iterator to all map elements of this current box id
	put keypoint matches of this box id in a set for easy comparison
	for every keypoint match of this box id, go through all the current frame box id's
		compare which keypoint matches correspond in the prev and current frame
		if it has more elements incommon than the previous run
			store the common keypoint matches size
			store the current frame bounding box
	if the best bounding box match for prev frame has been found in current frame
		insert an element to the map bbBestMatches
		
FP.2
implemented computeTTCLidar in camFusion_Studen.cpp
for every point in the prev lidar cloud
	check that the reflectivity is larger than 0.4
		register the min x distance of prev frame
for every point in the prev lidar cloud
	check that the reflectivity is larger than 0.4
		register the min x distance of curr frame
		
Calculate the TTC
		
FP.3&4
implemented computeTTCCamera in camFusion_Studen.cpp
______________________
general remarks:
- I did not implement clusterKptMatchesWithRIO as this seemed
redundant from what I did in FP.1. See that section for this 
implementation
- I defined a valid distance as being larger than 40pix, and a ratio between curr and prev frame distance that changed within 5%
_________________________________
Step1: Associate keypoint matches to bounding boxes
This step was already implemented in FP.1

Step2:
for every keypoint match in the outer for loop
	register the prev and current keypoint
	for every keypoint n the inner for loop
		register the prev and current keypoint
		calculate the distance between the outer and inner keypoints
		calculate the distance ratio 
		check for valid values:
			distance in curr & prev frame is larger than mindistnace
			the distRatio is >= 0.95 and <= 1.05
				if valid
					add to valid distRatio's vector
					
Step3:
calculate the mean of these valid distRatios and calculate TTC

FP.5
I added a summary of my results in 3DObjectDetTTC.xlsx

Performance evaluation of TTC lidar
I think that in the first 4 frames (0 -> 3) we brake harder than the car in front of us.
I say this based on the delta x values that become lower, so the relative speed is lowering.
I think that explains why the TTC is increasing even though we get closer.

between frame pair 4 and 5 (3&4) we see a sudden change in values
It seems to be caused by a single lidar point that deviates from the others
See the picture 3D Objects4.jpg

The change to frame 5 also has that single lidar point, so the distance seems to be in line with the other measurements again

There is again a large deviation from frame 6 to 7, the delta x is suddently a few cm lower.
This causes the TTC to shoot up significantly.

The whole TTC calculation is also difficult to do at low relative speeds. 
That causes the delta x to be very close to the velodyne accuracy of 2cm.
It might also be caused by tilting of the lidar caused by braking.

Between frame 10&11 we have a huge deviation in delta x
This is again caused by one outlier in frame 11.
I think we might need better outlier detection. maybe lower the distance tolerance during eucidean clustering 

If we average over all the frames 0->18 I calculated an average speed of 0.64444 meters per second.
1.16 meter in 1.8s -> 0.6444mps
with an average distance of 7.5m, that would give a TTC of 11.6s.

Performance evaluation of TTC Camera
In general it looks like hte camera gives us a better measurement.
This is because the closer the camera gets, the better the measurement is.
the keypoints spans more pixels, with a better accuracy as result.

This process also suffers from these low speeds, the variation in distances are below 1%
--------------------------------------
I think I would have to appy a tighter euclidean clustering for the TTC lidar
We might have to rely more on the camera the closer we get to the object combined with lower relative speed.



