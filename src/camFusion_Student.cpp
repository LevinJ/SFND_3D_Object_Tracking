
#include <iostream>
#include <algorithm>
#include <numeric>
#include <utility>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

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


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size2f worldSize, cv::Size imageSize, bool bWait)
{
	//to better visual lidar point top view, fix the starting world size as 6
	const float START_HEIGHT = 6.8;
	worldSize.height = worldSize.height - START_HEIGHT;
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
        if(it1->lidarPoints.size() < 3){
        	//skip the bounding box display if it contains less than three points.
        	continue;
        }
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-(xw- START_HEIGHT) * imageSize.height / worldSize.height) + imageSize.height;
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
        sprintf(str1, "track id=%d, #pts=%d", it1->trackID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 1, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 1, currColor);
        cout<<"xmin="<<xwmin<<",yw="<<ywmax-ywmin<<",id="<<it1->boxID<<",pts="<<it1->lidarPoints.size()<<endl;
    }

    // plot distance markers
//    float lineSpacing = 1.0; // gap between distance markers
//    int nMarkers = floor(worldSize.height / lineSpacing);
    int nMarkers = 10;
    float lineSpacing =  worldSize.height / float(nMarkers);

    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }



    if(bWait)
    {
    	// display image
		string windowName = "3D Objects";
		cv::namedWindow(windowName, 1);
		cv::imshow(windowName, topviewImg);
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
	float shrinkFactor = 0.10; // shrinks each bounding box by the given percentage to avoid 3D object merging at the edges of an ROI
    for(auto &m: kptMatches){
    	auto &pt = kptsCurr[m.trainIdx].pt;
    	// shrink current bounding box slightly to avoid having too many outlier points around the edges
		cv::Rect smallerBox;
		smallerBox.x = boundingBox.roi.x + shrinkFactor * boundingBox.roi.width / 2.0;
		smallerBox.y = boundingBox.roi.y + shrinkFactor * boundingBox.roi.height / 2.0;
		smallerBox.width = boundingBox.roi.width * (1 - shrinkFactor);
		smallerBox.height = boundingBox.roi.height * (1 - shrinkFactor);
    	if(smallerBox.contains(pt)){
    		boundingBox.keypoints.push_back(kptsCurr[m.trainIdx]);
    		boundingBox.kptMatches.push_back(m);
    	}
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
	// compute distance ratios between all matched keypoints
	    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
	    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
	    { // outer kpt. loop

	        // get current keypoint and its matched partner in the prev. frame
	        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
	        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

	        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
	        { // inner kpt.-loop

	            double minDist = 100.0; // min. required distance

	            // get next keypoint and its matched partner in the prev. frame
	            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
	            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

	            // compute distances and distance ratios
	            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
	            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

	            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
	            {
	            	// avoid division by zero
	                double distRatio = distCurr / distPrev;
	                distRatios.push_back(distRatio);
	            }
	        } // eof inner loop over all matched kpts
	    }     // eof outer loop over all matched kpts

	    // only continue if list of distance ratios is not empty
	    if (distRatios.size() == 0)
	    {
	        TTC = NAN;
	        return;
	    }

	    // compute camera-based TTC from distance ratios
	    double ratio = 0;
	    std::sort (distRatios.begin(), distRatios.end());
		long medIndex = floor(distRatios.size() / 2.0);
		ratio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex];

	    double dT = 1 / frameRate;
	    TTC = -dT / (1 - ratio);

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
	double dT = 1/frameRate;        // time between two measurements in seconds
	// find closest distance to Lidar points within ego lane
	double minXPrev = 1e9, minXCurr = 1e9;
//	for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
//	{
//
//		minXPrev = minXPrev > it->x ? it->x : minXPrev;
//	}
//
//	for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
//	{
//		minXCurr = minXCurr > it->x ? it->x : minXCurr;
//	}
	vector<double> XPevs;
	vector<double> XCurrs;
	for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
	{
		XPevs.push_back( it->x);
	}

	for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
	{
		XCurrs.push_back(it->x );
	}
	std::sort(XPevs.begin(), XPevs.end());
	std::sort(XCurrs.begin(), XCurrs.end());

	//There are invariably some outliers between ego car and the preceding car, so take the nth smallest value
	const int N = 7;
	if(XPevs.size()<=N){
		minXPrev = XPevs[0];
	}else{
		minXPrev = XPevs[N];
	}

	if(XCurrs.size() <= N){
		minXCurr = XCurrs[0];
	}else{
		minXCurr = XCurrs[N];
	}

	cout<<"Lidar ttc, adjusted minXPrev="<<minXPrev<<",original minXPrev="<<XPevs[0]<<endl;
	cout<<"Lidar ttc, adjusted minXCurr="<<minXCurr<<",original minXCurr="<<XCurrs[0]<<",gap="<<minXCurr- XCurrs[0]<<endl;

	// compute TTC from both measurements
	TTC = minXCurr * dT / (minXPrev - minXCurr);
}

static bool find_containing_box(std::vector<BoundingBox>  & boxes, cv::Point2f &pt, int &boxid){
	int match_times = 0;
	for(int i=0; i< boxes.size(); i++){
		auto &box = boxes[i];
		if(box.roi.contains(pt)){
			boxid = i;
			match_times ++;
		}
	}
	if(match_times != 1){
		//assign a pt only if it belongs to just one bounding box
		boxid = -1;
	}
	return match_times == 1;
}
static int find_max_ind(const vector<int> &vec){
	return std::distance(vec.begin(), std::max_element(vec.begin(), vec.end()));
}

void show_kpt_matching(BoundingBox &boundingBox, DataFrame &prevFrame, DataFrame &currFrame){
	// visualize results
	cv::Mat matchImg;
	cv::drawMatches(prevFrame.cameraImg, prevFrame.keypoints, currFrame.cameraImg, currFrame.keypoints, boundingBox.kptMatches,
					matchImg, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
	cv::resize(matchImg, matchImg, cv::Size(0,0), 0.7,0.7);

	string windowName = "Matching keypoints between two camera images";
	cv::namedWindow(windowName, 7);
	cv::imshow(windowName, matchImg);
	cv::waitKey(0);
}

void show_bd_matching(std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame){
	cv::Mat prev_img = prevFrame.cameraImg.clone();
	cv::Mat curr_img = currFrame.cameraImg.clone();
	std::vector<std::pair<cv::Rect, cv::Rect>> res_vec;



	for(auto &match : bbBestMatches){
		cv::Rect prev_rect;
		cv::Rect curr_rect;
		for(auto &box:prevFrame.boundingBoxes ){
			if(box.boxID == match.first){
				prev_rect = box.roi;
			}
		}
		for(auto &box: currFrame.boundingBoxes){
			if(box.boxID == match.second){
				curr_rect = box.roi;
			}
		}

		prev_rect.y += curr_img.rows;
		res_vec.push_back(std::make_pair(curr_rect, prev_rect));

	}



	cv::Mat visImg;
	cv::vconcat(curr_img, prev_img, visImg);

	//draw the bounding box and connection line
	int i = 0;
	for(auto &m: res_vec){
		auto &curr_rect = m.first;
		auto &prev_rect =  m.second;

		cv::RNG rng(i++);
		cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));
		cv::rectangle(visImg, prev_rect, currColor, 2);
		cv::rectangle(visImg, curr_rect, currColor, 2);

		cv::Point pt1(curr_rect.x + curr_rect.width/2, curr_rect.y + curr_rect.height/2);
		cv::Point pt2(prev_rect.x + prev_rect.width/2, prev_rect.y + prev_rect.height/2);
		cv::line(visImg, pt1, pt2, currColor, 3);
	}
	cout<<"prev frame "<<prevFrame.img_file<<" = "<<prevFrame.boundingBoxes.size()<<endl;
	cout<<"curr frame "<<currFrame.img_file<<" = "<<currFrame.boundingBoxes.size()<<endl;
	cout<< "There are "<<res_vec.size()<<" matched box pair"<<endl;

	// display image
	string windowName = "boundign box matching";
	cv::namedWindow(windowName, 1);
	cv::imshow(windowName, visImg);
	cv::waitKey(0); // wait for key to be pressed

}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // ...
	vector<vector<int>> match_matrix;
	const int ROW = prevFrame.boundingBoxes.size();
	const int COL = currFrame.boundingBoxes.size();
//	int match_matrix[ROW][COL];
	 for(int i=0; i<ROW; i++){
		 vector<int> row_vec(COL,0);
		 match_matrix.push_back(row_vec);
	 }

	for(auto &m:matches){
		int src_boxid = -1;
		//check which box the src_pt belongs to
		if(!find_containing_box(prevFrame.boundingBoxes, prevFrame.keypoints[m.queryIdx].pt, src_boxid)) continue;
		//check which box the ref_pt belongs to
		int ref_boxid = -1;
		if(!find_containing_box(currFrame.boundingBoxes, currFrame.keypoints[m.trainIdx].pt, ref_boxid)) continue;
		//match has been found right in the two bounding box, fill in the match matrix
		match_matrix[src_boxid][ref_boxid]++;
	}

	for(int i=0; i< ROW; i++){
		int j = find_max_ind(match_matrix[i]);
		if(match_matrix[i][j] > 0){
			bbBestMatches[prevFrame.boundingBoxes[i].boxID] = currFrame.boundingBoxes[j].boxID;
			//here to simplify things, we set all track id as above 1000
			const int START_TRACKID = 1000;
			if(prevFrame.boundingBoxes[i].trackID < START_TRACKID){
				prevFrame.boundingBoxes[i].trackID += START_TRACKID;
			}
			currFrame.boundingBoxes[j].trackID = prevFrame.boundingBoxes[i].trackID;
		}

	}
}
