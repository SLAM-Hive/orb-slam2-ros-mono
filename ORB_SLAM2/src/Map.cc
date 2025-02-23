/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

// 保存地图时，保存为pcd格式，需要的信息如下
/**
    # .PCD v0.7 - Point Cloud Data file format
    VERSION 0.7
    FIELDS x y z
    SIZE 4 4 4
    TYPE F F F
    COUNT 1 1 1
    WIDTH 512
    HEIGHT 1
    VIEWPOINT 0 0 0 1 0 0 0
    POINTS 2
    DATA ascii
    -0.00529 0.1235 0.8473
    0.03254 -0.1234 0.9312
*/
void Map::Save ( const string& filename )
 {
    cerr<<"Map Saving to "<<filename <<endl;
    unsigned long int nMapPoints = mspMapPoints.size();

    ofstream f;
    f.open(filename.c_str());
    //写入基本信息
    f << "# .PCD v0.7 - Point Cloud Data file format" << endl;
    f << "VERSION 0.7" << endl;
    f << "FIELDS x y z" << endl;
    f << "SIZE 4 4 4" << endl;
    f << "TYPE F F F" << endl;
    f << "COUNT 1 1 1" << endl;
    f << "WIDTH " << nMapPoints << endl;
    f << "HEIGHT 1" << endl;
    f << "VIEWPOINT 0 0 0 1 0 0 0" << endl;
    f << "POINTS " << nMapPoints << endl;
    f << "DATA ascii" << endl;
    
    // 写入点云数据
    int temp_flag = 0;
    for ( auto mp: mspMapPoints )
    {
        temp_flag++;

        //遍历每一个点
        cv::Mat mpWorldPos = mp->GetWorldPos();
        f << mpWorldPos.at<float>(0) << " " << mpWorldPos.at<float>(1) << " " << mpWorldPos.at<float>(2);
        if(temp_flag != nMapPoints) f << endl;
             
    }
    f.close();


    cerr<<"Map Saving Finished!"<<endl;
 }
} //namespace ORB_SLAM
