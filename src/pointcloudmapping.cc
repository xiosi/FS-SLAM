/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "pointcloudmapping.h"
#include <KeyFrame.h>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/projection_matrix.h>
#include "Converter.h"
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

#include <boost/make_shared.hpp>



float cloudCurvature[500000];
int cloudSortInd[500000];
int cloudNeighborPicked[500000];
int cloudLabel[500000];



bool comp(int i, int j) { return (cloudCurvature[i] < cloudCurvature[j]); }



PointCloudMapping::PointCloudMapping(double resolution_)
{
    this->resolution = resolution_;
    voxel.setLeafSize( resolution, resolution, resolution);
    globalMap = boost::make_shared< PointCloud >( );

    // viewerThread = make_shared<thread>( bind(&PointCloudMapping::viewer, this ) );
}

void PointCloudMapping::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
        keyFrameUpdated.notify_one();
    }
    viewerThread->join();
}

void PointCloudMapping::insertKeyFrame(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    // cout<<"receive a keyframe, id = "<<kf->mnId<<endl;
    unique_lock<mutex> lck(keyframeMutex);
    keyframes.push_back( kf );
    colorImgs.push_back( color.clone() );
    depthImgs.push_back( depth.clone() );

    keyFrameUpdated.notify_one();
}

std::vector<pcl::PointCloud< PointCloudMapping::PointT >::Ptr> PointCloudMapping::generatePointCloud(KeyFrame* kf, cv::Mat& color, cv::Mat& depth)
{
    
    // point cloud is null ptr

    std::vector<PointCloud> scanCloud;
    for ( int m=0; m<depth.rows; m+=7)
    {
        PointCloud::Ptr temp( new PointCloud() );
        for ( int n=0; n<depth.cols; n+=2 )
        {
            float d = depth.ptr<float>(m)[n];
            if (d < 0.01 || d>10)
                continue;
            PointT p;
            p.z = d;
            p.x = ( n - kf->cx) * p.z / kf->fx;
            p.y = ( m - kf->cy) * p.z / kf->fy;

            // p.b = color.ptr<uchar>(m)[n*3];
            // p.g = color.ptr<uchar>(m)[n*3+1];
            // p.r = color.ptr<uchar>(m)[n*3+2];

            temp->points.push_back(p);
        }      
        scanCloud.push_back(*temp);
    }
    std::vector<int> scanStartInd(int(scanCloud.size()), 0);
    std::vector<int> scanEndInd(int(scanCloud.size()), 0);
    scanStartInd[0]=5;
    scanEndInd[0]=int(scanCloud[0].size())-6;
    
    for (int i=1; i<int(scanCloud.size()); i++){
        scanStartInd[i]=scanEndInd[i-1]+6+5;
        scanEndInd[i]=scanEndInd[i-1]+6+int(scanCloud[i].size())-6;
        // std::cout<<"yichu:"<<scanStartInd[i]<<","<<scanEndInd[i]<<std::endl;
    }
    // std::cout<<"yichu"<<int(scanCloud.size())<<std::endl;
    for(int i=0;i<int(scanCloud.size());i++){
        // std::cout<<"yichu"<<scanCloud[6].size()<<std::endl;
        // std::cout<<"yichu:"<<scanStartInd[i]<<","<<scanEndInd[i]<<","<<int(scanCloud[i].size())<<std::endl;
        for(int j=5;j<int(scanCloud[i].size())-5;j++){
            // std::cout<<"jin lai le:"<<i<<" "<<int(scanCloud[i].size())-5<<std::endl;
            float diffX = scanCloud[i].points[j - 5].x + scanCloud[i].points[j - 4].x + scanCloud[i].points[j - 3].x + scanCloud[i].points[j - 2].x + scanCloud[i].points[j - 1].x - 10 * scanCloud[i].points[j].x + scanCloud[i].points[j + 1].x + scanCloud[i].points[j + 2].x + scanCloud[i].points[j + 3].x + scanCloud[i].points[j + 4].x + scanCloud[i].points[j + 5].x;
            float diffY = scanCloud[i].points[j - 5].y + scanCloud[i].points[j - 4].y + scanCloud[i].points[j - 3].y + scanCloud[i].points[j - 2].y + scanCloud[i].points[j - 1].y - 10 * scanCloud[i].points[j].y + scanCloud[i].points[j + 1].y + scanCloud[i].points[j + 2].y + scanCloud[i].points[j + 3].y + scanCloud[i].points[j + 4].y + scanCloud[i].points[j + 5].y;
            float diffZ = scanCloud[i].points[j - 5].z + scanCloud[i].points[j - 4].z + scanCloud[i].points[j - 3].z + scanCloud[i].points[j - 2].z + scanCloud[i].points[j - 1].z - 10 * scanCloud[i].points[j].z + scanCloud[i].points[j + 1].z + scanCloud[i].points[j + 2].z + scanCloud[i].points[j + 3].z + scanCloud[i].points[j + 4].z + scanCloud[i].points[j + 5].z;

            cloudCurvature[scanStartInd[i]+j-5] = diffX * diffX + diffY * diffY + diffZ * diffZ;
            cloudSortInd[scanStartInd[i]+j-5] = scanStartInd[i]+j-5;
            cloudNeighborPicked[scanStartInd[i]+j-5] = 0;
            cloudLabel[scanStartInd[i]+j-5] = 0;
            // std::cout<<"yichu:"<<scanStartInd[i]+j<<std::endl;
        }
        
    }
    // std::cout<<"jin lai le"<<scanCloud[6].size()<<std::endl;
    pcl::PointCloud< PointCloudMapping::PointT >cornerPointsSharp;
    pcl::PointCloud< PointCloudMapping::PointT >cornerPointsLessSharp;
    pcl::PointCloud< PointCloudMapping::PointT >surfPointsFlat;
    pcl::PointCloud< PointCloudMapping::PointT >surfPointsLessFlat;
    for (int i=0; i<int(scanCloud.size()); i++){
        if (scanEndInd[i] - scanStartInd[i] < 6)
            continue;
        pcl::PointCloud< PointCloudMapping::PointT>::Ptr surfPointsLessFlatScan(new pcl::PointCloud< PointCloudMapping::PointT>);
        for (int j=0; j<6; j++){
            //每个等分的起点和终点
            // std::cout<<"divide:"<<scanStartInd[i]<<","<<scanEndInd[i]<<std::endl;
            int sp = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * j / 6;
            int ep = scanStartInd[i] + (scanEndInd[i] - scanStartInd[i]) * (j + 1) / 6 - 1;
            //曲率大到小排序
            std::sort(cloudSortInd + sp, cloudSortInd + ep, comp);
            // std::cout<<"sort success:"<<sp<<","<<ep<<std::endl;
            int largestPickedNum = 0;
            // //挑选每个分段的曲率很大和比较大的点
            for (int k = ep; k >= sp; k--)
            {
                int ind = cloudSortInd[k]+5-scanStartInd[i];
                //如果筛选标志为0，并且曲率较大
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] > 0.1)
                {

                    largestPickedNum++;
                    //最大两个是角点
                    if (largestPickedNum <= 2)
                    {
                        cloudLabel[ind] = 2;
                        cornerPointsSharp.push_back(scanCloud[i].points[ind]);
                        cornerPointsLessSharp.push_back(scanCloud[i].points[ind]);
                    }
                    //最多保存20个特征点
                    else if (largestPickedNum <= 20)
                    {
                        cloudLabel[ind] = 1;
                        cornerPointsLessSharp.push_back(scanCloud[i].points[ind]);
                    }
                    else
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = scanCloud[i].points[ind + l].x - scanCloud[i].points[ind + l - 1].x;
                        float diffY = scanCloud[i].points[ind + l].y - scanCloud[i].points[ind + l - 1].y;
                        float diffZ = scanCloud[i].points[ind + l].z - scanCloud[i].points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = scanCloud[i].points[ind + l].x - scanCloud[i].points[ind + l + 1].x;
                        float diffY = scanCloud[i].points[ind + l].y - scanCloud[i].points[ind + l + 1].y;
                        float diffZ = scanCloud[i].points[ind + l].z - scanCloud[i].points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            int smallestPickedNum = 0;
            for (int k = sp; k <= ep; k++)
            {
                int ind = cloudSortInd[k]+5-scanStartInd[i];
                ////如果筛选标志为0，并且曲率较小
                if (cloudNeighborPicked[ind] == 0 &&
                    cloudCurvature[ind] < 0.1)
                {

                    cloudLabel[ind] = -1;
                    surfPointsFlat.push_back(scanCloud[i].points[ind]);

                    smallestPickedNum++;
                    //只取4个
                    if (smallestPickedNum >= 4)
                    {
                        break;
                    }

                    cloudNeighborPicked[ind] = 1;
                    //同样稀疏特征点分布
                    for (int l = 1; l <= 5; l++)
                    {
                        float diffX = scanCloud[i].points[ind + l].x - scanCloud[i].points[ind + l - 1].x;
                        float diffY = scanCloud[i].points[ind + l].y - scanCloud[i].points[ind + l - 1].y;
                        float diffZ = scanCloud[i].points[ind + l].z - scanCloud[i].points[ind + l - 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                    for (int l = -1; l >= -5; l--)
                    {
                        float diffX = scanCloud[i].points[ind + l].x - scanCloud[i].points[ind + l + 1].x;
                        float diffY = scanCloud[i].points[ind + l].y - scanCloud[i].points[ind + l + 1].y;
                        float diffZ = scanCloud[i].points[ind + l].z - scanCloud[i].points[ind + l + 1].z;
                        if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05)
                        {
                            break;
                        }

                        cloudNeighborPicked[ind + l] = 1;
                    }
                }
            }
            //将剩余的点（包括之前被排除的点）全部归入平面点中less flat类别
            for (int k = sp; k <= ep; k++)
            {
                if (cloudLabel[k] <= 0)
                {
                    int ind=k+5-scanStartInd[i];
                    surfPointsLessFlatScan->push_back(scanCloud[i].points[ind]);
                }
            }
        }
        //由于less flat点最多，对每个分段less flat的点进行体素栅格滤波
        pcl::PointCloud< PointCloudMapping::PointT> surfPointsLessFlatScanDS;
        pcl::VoxelGrid< PointCloudMapping::PointT> downSizeFilter;
        downSizeFilter.setInputCloud(surfPointsLessFlatScan);
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilter.filter(surfPointsLessFlatScanDS);

        surfPointsLessFlat += surfPointsLessFlatScanDS;
    }
    pcl::PointCloud< PointCloudMapping::PointT > outCloud;

    //下采样
    // for (int i=0;i<int(scanCloud.size());i++){
    //     outCloud+=scanCloud[i];
    // }

    PointCloud::Ptr tmp( new PointCloud() );
    PointCloud::Ptr cornerPointsSharpPtr( new PointCloud() );
    PointCloud::Ptr cornerPointsLessSharpPtr( new PointCloud() );
    PointCloud::Ptr surfPointsFlatPtr( new PointCloud() );
    PointCloud::Ptr surfPointsLessFlatPtr( new PointCloud() );
    //特征
    *cornerPointsSharpPtr=cornerPointsSharp;
    *cornerPointsLessSharpPtr=cornerPointsLessSharp;
    *surfPointsFlatPtr=surfPointsFlat;
    *surfPointsLessFlatPtr=surfPointsLessFlat;
    // *tmp=outCloud;
    std::vector<pcl::PointCloud< PointCloudMapping::PointT >::Ptr> mpCloud;


    Eigen::Isometry3d T = ORB_SLAM2::Converter::toSE3Quat( kf->GetPose() );
    // PointCloud::Ptr cloud(new PointCloud);
    // pcl::transformPointCloud( *tmp, *cloud, T.inverse().matrix());
    // cloud->is_dense = false;
    PointCloud::Ptr cloud1(new PointCloud);
    pcl::transformPointCloud( *cornerPointsSharpPtr, *cloud1, T.inverse().matrix());
    cloud1->is_dense = false;
    PointCloud::Ptr cloud2(new PointCloud);
    pcl::transformPointCloud( *cornerPointsLessSharpPtr, *cloud2, T.inverse().matrix());
    cloud2->is_dense = false;
    PointCloud::Ptr cloud3(new PointCloud);
    pcl::transformPointCloud( *surfPointsFlatPtr, *cloud3, T.inverse().matrix());
    cloud3->is_dense = false;
    PointCloud::Ptr cloud4(new PointCloud);
    pcl::transformPointCloud( *surfPointsLessFlatPtr, *cloud4, T.inverse().matrix());
    cloud4->is_dense = false;
    mpCloud.push_back(cloud1);
    mpCloud.push_back(cloud2);
    mpCloud.push_back(cloud3);
    mpCloud.push_back(cloud4);

    // cout<<"generate point cloud for kf "<<kf->mnId<<", size="<<cloud->points.size()<<endl;
    //取消render
    // render(cloud);
    return mpCloud;
}

pcl::PointCloud< PointCloudMapping::PointT >::Ptr  PointCloudMapping::MergePointCloud(std::vector<pcl::PointCloud< PointCloudMapping::PointT >::Ptr> mpCloud){
    PointCloud::Ptr dstC( new PointCloud() );
    pcl::PointCloud< PointCloudMapping::PointT > outCloud;
    for(size_t  i=0; i< mpCloud.size(); i++){
        outCloud+=(*mpCloud[i]);
    }
    *dstC=outCloud;
    render(dstC);
    return dstC;
}


void PointCloudMapping::viewer()
{
    pcl::visualization::CloudViewer viewer("viewer");
    while(1)
    {
        {
            unique_lock<mutex> lck_shutdown( shutDownMutex );
            if (shutDownFlag)
            {
                break;
            }
        }
        {
            unique_lock<mutex> lck_keyframeUpdated( keyFrameUpdateMutex );
            keyFrameUpdated.wait( lck_keyframeUpdated );
        }

        // keyframe is updated
        size_t N=0;
        {
            unique_lock<mutex> lck( keyframeMutex );
            N = keyframes.size();
        }

        for ( size_t i=lastKeyframeSize; i<N ; i++ )
        {
            PointCloud::Ptr p = MergePointCloud(generatePointCloud( keyframes[i], colorImgs[i], depthImgs[i] ));
            *globalMap += *p;
        }
        // render(globalMap);
        pcl::io::savePCDFileBinary("vslam.pcd", *globalMap); 
        PointCloud::Ptr tmp(new PointCloud());
        voxel.setInputCloud( globalMap );
        voxel.filter( *tmp );
        globalMap->swap( *tmp );
        viewer.showCloud( globalMap );
        // cout << "show global map, size=" << globalMap->points.size() << endl;
        lastKeyframeSize = N;
    }
}


std::vector<pcl::PointCloud< PointCloudMapping::PointT >::Ptr> PointCloudMapping::getCurPointCloud(){
    // keyframe is updated
    size_t N=0;
    {
        unique_lock<mutex> lck( keyframeMutex );
        N = keyframes.size();
    }
    std::vector<pcl::PointCloud< PointCloudMapping::PointT >::Ptr> pc;
    if (N>0)
    return generatePointCloud( keyframes[N-1], colorImgs[N-1], depthImgs[N-1] );    
    return pc;
}

cv::Mat PointCloudMapping::getImd(){
    // keyframe is updated
    // size_t N=0;
    // {
    //     unique_lock<mutex> lck( keyframeMutex );
    //     N = keyframes.size();
    // }
    cv::Mat im;
    if (keyframes.size()>0){
        depthImgs[keyframes.size()-1].copyTo(im);
    }
    return im;
}

void PointCloudMapping::render(pcl::PointCloud<PointT>::Ptr cloud)
{
    
	string orientation;
	// cout << "->请输入按哪一坐标字段渲染赋色（x/y/z）：";
	// cin >> orientation;
	// cout << "->正在按" << orientation << "字段渲染赋色..." << endl;
	PointT min;
	PointT max;
	pcl::getMinMax3D(*cloud, min, max);
    orientation = "x";
	float L;
	float L_2;
	if (orientation == "x")
	{
    
		L = max.x - min.x;
		L_2 = L / 2.0;

		for (size_t i = 0; i < cloud->size(); ++i)
		{
    
			if ((cloud->points[i].x - min.x) < L_2)
			{
    
				cloud->points[i].r = 0;
				cloud->points[i].g = (255 * ((cloud->points[i].x - min.x) / L_2));
				cloud->points[i].b = (255 * (1 - ((cloud->points[i].x - min.x) / L_2)));
			}
			else
			{
    
				cloud->points[i].r = (255 * (cloud->points[i].x - min.x - L_2) / L_2);
				cloud->points[i].g = (255 * (1 - (cloud->points[i].x - min.x - L_2) / L_2));
				cloud->points[i].b = 0;
			}
		}
	}
	else if (orientation == "y")
	{
    
		L = max.y - min.y;
		L_2 = L / 2.0;
		for (size_t i = 0; i < cloud->size(); ++i)
		{
    
			if ((cloud->points[i].y - min.y) < L_2)
			{
    
				cloud->points[i].r = 0;
				cloud->points[i].g = (255 * ((cloud->points[i].y - min.y) / L_2));
				cloud->points[i].b = (255 * (1 - ((cloud->points[i].y - min.y) / L_2)));
			}
			else
			{
    
				cloud->points[i].r = (255 * (cloud->points[i].y - min.y - L_2) / L_2);
				cloud->points[i].g = (255 * (1 - (cloud->points[i].y - min.y - L_2) / L_2));
				cloud->points[i].b = 0;
			}
		}
	}
	else if (orientation == "z")
	{
    
		L = max.z - min.z;
		L_2 = L / 2.0;
		for (size_t i = 0; i < cloud->size(); ++i)
		{
    
			if ((cloud->points[i].z - min.z) < L_2)
			{
    
				cloud->points[i].r = 0;
				cloud->points[i].g = (255 * ((cloud->points[i].z - min.z) / L_2));
				cloud->points[i].b = (255 * (1 - ((cloud->points[i].z - min.z) / L_2)));
			}
			else
			{
    
				cloud->points[i].r = (255 * (cloud->points[i].z - min.z - L_2) / L_2);
				cloud->points[i].g = (255 * (1 - (cloud->points[i].z - min.z - L_2) / L_2));
				cloud->points[i].b = 0;
			}
		}
	}
	else
	{
    
		PCL_ERROR("警告！请输入小写字母下的坐标字段（ x 或 y 或 z ）\a\n");
	}
}

