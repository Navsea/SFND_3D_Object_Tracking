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
Performance evaluation of TTC lidar
between frame pair 3 and 4 we see a sudden change in values
From the pictures i can see that we suddenly use different lidar points.
Could be caused by the fact that we are getting close to the car and get a new perspective.
or maybe different lighting conditions?


by only taking keypoint match pairs that change within a range, this seems to be filtered outer

1.16 meter in 1.8s -> 0.6444mps
kmph: 2.32kph


