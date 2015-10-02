#include <pcl/io/ply_io.h>

#include "tools.h"

int main(int argc, char *argv[]) {
	
	if (argc < 2) {
		printf("%s << ERROR! No input PLY (mesh/cloud) file has been provided.\n", __FUNCTION__);
		return 1;
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PolygonMesh::Ptr mesh;
	mesh = pcl::PolygonMesh::Ptr(new pcl::PolygonMesh());

	char input_ply_name[256];
	sprintf(input_ply_name, "%s", argv[1]);

	if (pcl::io::loadPLYFile(input_ply_name, *mesh) < 0) {
		printf("%s << ERROR! Loading of file (%s) failed.\n", __FUNCTION__, input_ply_name);
		return 1;
	}
	fromPCLPointCloud2(mesh->cloud, *cloud);

	cv::Mat depth;

	int cols = 640, rows = 480;

	depth = cv::Mat::zeros(rows, cols, CV_16UC1);

	double fx = 570.6, fy = 558.8;
	double cx = 320.7, cy = 243.0;

	int iii, jjj;
	for (int xxx = 0; xxx < cloud->points.size(); xxx++) {
		
		iii = float2int((cloud->points[xxx].x*fx)/float(-cloud->points[xxx].z) + cx);
		jjj = float2int((-cloud->points[xxx].y*fy)/float(-cloud->points[xxx].z) + cy);
	
		if ((iii < cols) && (iii >= 0) && (jjj < rows) && (jjj >= 0)) {
			int depth_val = -cloud->points[xxx].z*DEFAULT_LEVELS_PER_MM;
			if ((depth.at<unsigned short>(jjj,iii) == 0) || (depth_val < depth.at<unsigned short>(jjj,iii))) { // Only fill pixel if current point lies closer to the camera than previous
				depth.at<unsigned short>(jjj,iii) = depth_val; // is this correct??
			}
			
		} else {
			printf("%s << Dodgy point: (%d, %d) from (%f, %f, %f)\n", __FUNCTION__, iii, jjj, cloud->points[xxx].x, cloud->points[xxx].y, cloud->points[xxx].z);
		}
	}

	char output_png_name[256];
	sprintf(output_png_name, "%s_depth.png", input_ply_name);
	cv::imwrite(output_png_name, depth);
	
	return 0;
	
}
