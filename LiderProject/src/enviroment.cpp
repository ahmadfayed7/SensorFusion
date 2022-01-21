/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "render/render.h"
#include <unordered_set>
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"
#include "kdtree.h"
pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	float x1,x2,x3,y1,y2,y3,z1,z2,z3;
for (int i=0 ;i<maxIterations;i++)
{
	std::unordered_set<int> inliers;
		while (inliers.size() < 3)
		{
			inliers.insert(rand() % (cloud->points.size()));
		}
		
 auto iterator = inliers.begin();
  x1 =cloud->points[*iterator].x;
  y1 =cloud->points[*iterator].y; 
  z1 =cloud->points[*iterator].z; 
  iterator++;
  x2 =cloud->points[*iterator].x;
  y2 =cloud->points[*iterator].y; 
  z2 =cloud->points[*iterator].z;  
  iterator++;
  x3 =cloud->points[*iterator].x;
  y3 =cloud->points[*iterator].y; 
  z3 =cloud->points[*iterator].z;  

  		float a = (y2 - y1)*(z3-z1) -(z2-z1)*(y3-y1);
		float b = (z2-z1)*(x3-x1) -(x2-x1)*(z3-z1);
		float c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); 
        float d = -(a*x1+b*y1+c*z1) ;
	for(int index=0;index <cloud->points.size();index++)

    {
			
		float x3 = cloud->points[index].x;
		float y3 = cloud->points[index].y;
		float z3 = cloud->points[index].z;
		float distance = fabs(a*x3 +b*y3+c*z3+d)/ sqrt(a*a + b*b+c*c);
		if (distance <=distanceTol)
		{
			inliers.insert(index);
		}
    }
		if (inliers.size() > inliersResult.size())
		{
			inliersResult.clear();
			inliersResult = inliers;
		}
	
}
	return inliersResult;

}




void euclideanCluster_helper(typename pcl::PointCloud<pcl::PointXYZI>::Ptr cluster,pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, KdTree* tree, float distanceTol, std::vector<bool> &processed ,int index,int maxSize )
{
	   if ((processed[index] == false) && (cluster->points.size() < maxSize))
    {
   processed [index]=true;
   cluster->points.push_back(inputCloud->points[index]);
    std::vector<int> nearby = tree->search(inputCloud->points[index],distanceTol);
  	for(int i : nearby)
      {
        if(!processed[i])
		  	euclideanCluster_helper(cluster,inputCloud, tree, distanceTol,processed,i,maxSize);

	  }
	}

}
std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanCluster(typename pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud, float distanceTol, int minSize, int maxSize)
{

	 KdTree* tree = new KdTree;
   
    for (int i=0; i<inputCloud->points.size(); i++) 
    	tree->insert(inputCloud->points[i],i); 
    
	std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;
	std ::vector<bool> processed (inputCloud->points.size() ,false);
	 for (int i =0;i<inputCloud->points.size();i++)
	 {
		if(processed[i]==true)
     	   continue;
	    typename pcl::PointCloud<pcl::PointXYZI>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZI>() );
	 	euclideanCluster_helper(cluster,inputCloud, tree, distanceTol,processed,i, maxSize);
               
			if ((cluster->points.size() >= minSize) && (cluster->points.size() <= maxSize))
            {
                clusters.push_back(cluster);
            }

	 }
 
	return clusters;

}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessor,  pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud)
{
	inputCloud =pointProcessor.FilterCloud(inputCloud, 0.15 , Eigen::Vector4f (-20,-6,-2,1), Eigen::Vector4f (30,7,5,1));
   std::unordered_set<int> inliers = Ransac(inputCloud, 50,0.2);

   typename pcl::PointCloud<pcl::PointXYZI>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZI>());
	typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZI>());

	for(int index = 0; index < inputCloud->points.size(); index++)
	{
		pcl::PointXYZI point = inputCloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
 		
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));

 

std::vector <pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = euclideanCluster(cloudOutliers, 0.42, 18, 1500);
   int clusterId = 0;
std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
{
      std::cout << "cluster size ";
      pointProcessor.numPoints(cluster);
      renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId%3]);
      Box box = pointProcessor.BoundingBox(cluster);
     renderBox(viewer,box,clusterId);
      ++clusterId;
}
   

    
}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}



int main ()
{


    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

	  ProcessPointClouds<pcl::PointXYZI> pointProcessorI ;//= new ProcessPointClouds<pcl::PointXYZI>();
     std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
     auto streamIterator = stream.begin();
     pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;


    while (!viewer->wasStopped ())
{

  // Clear viewer
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  	  // Load pcd and run obstacle detection process
  inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
  cityBlock(viewer, pointProcessorI, inputCloudI);

  streamIterator++;
  if(streamIterator == stream.end())
    streamIterator = stream.begin();

  viewer->spinOnce ();
}
  	
}
