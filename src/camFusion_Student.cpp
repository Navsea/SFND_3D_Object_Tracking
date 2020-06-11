
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    static int img_count = 0;
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);
    cv::imwrite(windowName + std::to_string(img_count) + ".jpg", topviewImg);
    img_count++;

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // it seemed wasteful to implement this function
    // the keypoint match with bb association was already done in the matchBoundingBoxes function
    // to find outliers you need to calculate all the distances, which you would have to do again in the TTC function.
    // im not sure how Udacity envisioned the implementation

}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // the association of keypointmatches and bb's is already done in the matchBoundingBoxes function
    // now only checking for outliers
    double distPrev, distCurr;
    std::vector<double> distRatios;

    cout << "# kptMatches: " << kptMatches.size() << endl;
    // calculate all the valid distances between keypoints
    for (cv::DMatch matchOuter: kptMatches)
    {
        cv::KeyPoint kpOuterPrev = kptsPrev[matchOuter.queryIdx];
        cv::KeyPoint kpOuterCurr = kptsCurr[matchOuter.trainIdx];

        for (cv::DMatch matchInner: kptMatches) 
        {
            double minDist = 40;
            cv::KeyPoint kpInnerPrev = kptsPrev[matchInner.queryIdx];
            cv::KeyPoint kpInnerCurr = kptsCurr[matchInner.trainIdx];

            // calculate the distances and distance ratio's
            distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);
            distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            //cout << "distPrev: " << distPrev << " distCurr: " << distCurr << endl;

            /// make sure that the distances are big enough to do calculations on and that the ratio is within 5%
            double distRatio = distCurr/distPrev;
            if((distCurr > minDist) && (distPrev > minDist) && (distRatio >= 0.95) && (distRatio <= 1.05))
            {
                //cout << "push it real good!" << endl;
                distRatios.push_back(distCurr/distPrev);
            }
        }
    }
    // print valid distances
    /*
    cout << "Valid distanceRatios: ";
    for(double distRatio: distRatios)
        cout << distRatio << ", ";
    cout << endl;*/

    // calculate the mean of all distances
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();
    cout << "meanDistRatio: " << meanDistRatio << endl;

    /*
    // remove all the outliers from kptMatches, we assume that the distance ratio does not change by more than 15% between frames
    double inlierOffset = 0.15; 
    for (auto it = distRatios.begin(); it != distRatios.end(); it++)
    {
        if( (*it > (meanDistRatio + inlierOffset)) && (*it < (meanDistRatio - inlierOffset)))
        {
            distRatios.erase(it);
            it--; // this is needed or your for loop wil go out of bounds
        }
    }
    */

    // recalc mean without outliers
    //meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0)  / distRatios.size();

    // calc TTC
    TTC = -(1/frameRate)/(1-meanDistRatio);
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    // filter the cloud based on reflectivity
    // determine the closest points and calculate TTC
    double minXPrev = 1000, minXCurr = 1000;
    double minReflec = 0.4;
    static int count = 0;
    for(auto point: lidarPointsPrev)
    {
        if(point.r > minReflec)
            minXPrev = (point.x < minXPrev)?point.x:minXPrev;
    }
    for(auto point: lidarPointsCurr)
    {
        if(point.r > minReflec)
            minXCurr = (point.x < minXCurr)?point.x:minXCurr;
    }

    TTC = (minXCurr * (frameRate/100)) / (minXPrev - minXCurr);

    cout << "frameRate" + std::to_string(count)+":" << frameRate << " minXPrev: " << minXPrev << " minXCurr: " << minXCurr << " TTC: " << TTC;
    count++;
}


void matchBoundingBoxes(DataFrame &prevFrame, DataFrame &currFrame, std::map<int, int> &bbBestMatches)
{
    // loop: by which bounding boxes are these keypoints enclosed. both prev and curr, those will be potential match candidates
    // which share the same bounding box id?
    std::multimap<int, int> prevFrame_bbMatches, currFrame_bbMatches;
    //std::multimap<int,int> bbMatches;

    /*
    std::cout << "#7': match bouding boxes" << std::endl;
    cout << "keypoint matches" << currFrame.kptMatches.size() << endl;

    for( auto it = currFrame.kptMatches.begin(); it != currFrame.kptMatches.end(); it++)
    {
        cout << "kid:" << it-currFrame.kptMatches.begin() << " qiD:" << it->queryIdx << " tiD:" << it->trainIdx << ", ";
    }
    cout << endl;
    */
    // step 1: loop over all keypoint matches and associate the respective keypoints with their bounding boxes
    for (auto it_km = currFrame.kptMatches.begin(); it_km != currFrame.kptMatches.end(); it_km++)
    {
        // for every keypoint check to which bounding box it belongs to (prevFrame)
        for (auto it_bb = prevFrame.boundingBoxes.begin(); it_bb != prevFrame.boundingBoxes.end(); it_bb++)
        {
            // Check if the keypoint match is located within this bounding box
            if (it_bb->roi.contains(prevFrame.keypoints[it_km->queryIdx].pt))
            {
                // not sure if this is needed but the field is still empty for sure
                it_bb->keypoints.push_back(prevFrame.keypoints[it_km->queryIdx]);
                it_bb->kptMatches.push_back(*it_km);

                // store the bounding box id as key together with the keypoint match index as value
                prevFrame_bbMatches.insert(std::pair<int, int> (it_bb->boxID, it_km-currFrame.kptMatches.begin()));
            }
        }
        
        // for every keypoint match check to which bounding box it belongs to (currFrame)
        for (auto it_bb = currFrame.boundingBoxes.begin(); it_bb != currFrame.boundingBoxes.end(); it_bb++)
        {
            if (it_bb->roi.contains(currFrame.keypoints[it_km->trainIdx].pt))
            {
                // not sure if this is needed but the field is still empty for sure
                it_bb->keypoints.push_back(currFrame.keypoints[it_km->trainIdx]);
                it_bb->kptMatches.push_back(*it_km);

                currFrame_bbMatches.insert(std::pair<int, int> (it_bb->boxID, it_km-currFrame.kptMatches.begin()));
            }
        }
    } 
    /*
    for (auto it_bb = prevFrame_bbMatches.begin(); it_bb != prevFrame_bbMatches.end(); it_bb++)
    {
        cout << "boxID: " << it_bb->first << " kmid: " << it_bb->second << " kid: " << currFrame.kptMatches[it_bb->second].queryIdx << endl;
    }
    for (auto it_bb = currFrame_bbMatches.begin(); it_bb != currFrame_bbMatches.end(); it_bb++)
    {
        cout << "boxID: " << it_bb->first << " kmid: " << it_bb->second << " kid: " << currFrame.kptMatches[it_bb->second].trainIdx << endl;
    }
    */
  	std::vector<std::pair<int,int>> bb_common_km;
    // for every bounding box of the prevFrame
    for(auto it_bb_pf = prevFrame.boundingBoxes.begin(); it_bb_pf != prevFrame.boundingBoxes.end(); it_bb_pf++)
    {	// get list to all km of this box ID, and iterate through all keypoints matches of this box
        // should maybe set list size beforehand (mem alloc speedup)
        std::set<int> pf_km_set;
        size_t common_set_count = 0;
        BoundingBox* best_bb_match = NULL; 
		std::pair<std::multimap<int,int>::iterator, std::multimap<int,int>::iterator> bb_km_range_pf = prevFrame_bbMatches.equal_range(it_bb_pf->boxID);
        for(auto it = bb_km_range_pf.first; it != bb_km_range_pf.second; it++)
            pf_km_set.insert(it->second);

        for(auto it = bb_km_range_pf.first; it != bb_km_range_pf.second; it++)
       
      	// iterate through all bounding boxes of the current frame
      	for(auto it_bb_cf = currFrame.boundingBoxes.begin(); it_bb_cf != currFrame.boundingBoxes.end(); it_bb_cf++)
      	{
            std::set<int> cf_km_set;
            std::set<int> common_km_set;
            //std::set<int> current_best_set;

        	// get list to all km of currFram
        	auto bb_km_range_cf = currFrame_bbMatches.equal_range(it_bb_cf->boxID);
            for(auto it = bb_km_range_cf.first; it != bb_km_range_cf.second; it++)
                cf_km_set.insert(it->second);

            // compare the two
            std::set_intersection(pf_km_set.begin(), pf_km_set.end(),cf_km_set.begin(), cf_km_set.end(), std::inserter(common_km_set,common_km_set.begin()));

            if(common_km_set.size()>common_set_count)
            {
                common_set_count = common_km_set.size();
                best_bb_match = &(*it_bb_cf);
                /*
                std::cout << "box id prevF: " << it_bb_pf-prevFrame.boundingBoxes.begin() << ", has these km incommon: ";
                for( int i: common_km_set)
                {
                    std::cout << i << ", ";
                }
                std::cout << " with currF: " << it_bb_cf-currFrame.boundingBoxes.begin() << std::endl;
                */
                //current_best_set = common_km_set;
            }
        }

        if(best_bb_match!=NULL)
            bbBestMatches.insert(std::pair<int,int>(it_bb_pf->boxID,  best_bb_match->boxID));
    }

    // visualize results
    bool bVis = false;
    if (bVis)
    {
        for (std::pair<int,int> pair: bbBestMatches)
            std::cout << "boxIDprev: " << pair.first << "boxIDcurr: " << pair.second << std::endl;

        cv::drawKeypoints(prevFrame.proc_img, prevFrame.keypoints, prevFrame.proc_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        for(auto it = prevFrame.keypoints.begin(); it!=prevFrame.keypoints.end(); it++)
        {
            string label = "kid:" + std::to_string(it - prevFrame.keypoints.begin());
            cv::putText(prevFrame.proc_img, label, it->pt, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));
        }
        string windowName = "Comparing images: prevFrame";
        cv::namedWindow(windowName, 6);
        imshow(windowName, prevFrame.proc_img);

        cv::drawKeypoints(currFrame.proc_img, currFrame.keypoints, currFrame.proc_img, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        for(auto it = currFrame.keypoints.begin(); it!=currFrame.keypoints.end(); it++)
        {
            string label = "kid:" + std::to_string(it - currFrame.keypoints.begin());
            cv::putText(currFrame.proc_img, label, it->pt, cv::FONT_HERSHEY_PLAIN, 2, cv::Scalar(0,0,255));
        }
        windowName = "Comparing images: currFrame";
        cv::namedWindow(windowName, 6);
        imshow(windowName, currFrame.proc_img);
        cv::waitKey();
    }
}
