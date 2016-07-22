// created by cgq.
// revised by czr, 07,06,2016.
// modified by cgq, 06/20/2016.
#include "mesh.h"

int main (int argc, char** argv)
{
	////**************************statistical outlier removal************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZ>);

    // Fill in the cloud data
    pcl::io::loadPCDFile ("tsdf_cloud6.pcd", *cloud);

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(0.5);
    sor.filter (*cloud_sor);
	
	double resolution = computeCloudResolution(cloud_sor);
	std::cout << "PointCloud after sor filtering: " << cloud_sor->points.size()
		<< " data points with resolution equal to " << resolution << "." << std::endl;

    //*******************************voxel grid filter*********************************
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;

    voxel_grid.setInputCloud(cloud_sor);
    double leaf = resolution * 4;
    voxel_grid.setLeafSize(leaf, leaf, leaf);			
    voxel_grid.filter(*cloud_filtered);
	 
	std::cout << "PointCloud after filtering: " << cloud_filtered->points.size() << std::endl;
	//pcl::copyPointCloud(*cloud_sor, *cloud_filtered);
	 pcl::PCDWriter writer;
     writer.write ("tsdf_cloud6_downsampled.pcd", *cloud_filtered, false);

	////*****************************Moving Least Square*******************************
	//// Create a KD-Tree
	//pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	//tree->setInputCloud(cloud_filtered);
	//// Output has the PointNormal type in order to store the normals calculated by MLS
	//pcl::PointCloud<pcl::PointNormal> mls_points;
	//// Init object (second point type is for the normals, even if unused)
	//pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

	//mls.setComputeNormals(true);

	//// Set parameters
	//mls.setInputCloud(cloud_filtered);
	//mls.setPolynomialFit(true);
	//mls.setPolynomialOrder(3);
	//mls.setSearchMethod(tree);
	//mls.setSearchRadius(0.005);

	//// Reconstruct
	//mls.process(mls_points);

	//pcl::PointCloud<pcl::PointNormal>::Ptr mls_points_ptr(new pcl::PointCloud<pcl::PointNormal>);
	//*mls_points_ptr = mls_points;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr xyz(new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::copyPointCloud(*mls_points_ptr, *xyz);


    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud);
	ne.setInputCloud(cloud);
    ne.setSearchMethod (tree2);
    ne.setKSearch (20);
	//ne.setRadiusSearch (0.005);
	Eigen::Vector4f centroid; 
    pcl::compute3DCentroid(*cloud, centroid); 
	ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
	std::cout<< "centroid 0:  " << centroid[0] << "centroid 1: " <<centroid[1] << "centroid 2:  "<< centroid[2] << std::endl;
    ne.compute (*normals);
	for(size_t i = 0; i < normals->size(); ++i){ 
       	normals->points[i].normal_x *= -1; 
       	normals->points[i].normal_y *= -1; 
      	normals->points[i].normal_z *= -1; 
       } 

    //* normals should not contain the point normals + surface curvatures

	std::cout << "test" << std::endl;

    // Concatenate the XYZRGB and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals
	
	

    //******************************Triangulation******************************************
    // Create search tree*

	/*double resolution2 = computeCloudResolution(tmp);
	std::cout << "PointCloud after mls: " << tmp->points.size()
		<< " data points with resolution equal to " << resolution2 << "." << std::endl;*/
	//PointCloud after mls: 127048 data points with resolution equal to 0.000974483.


    pcl::search::KdTree<pcl::PointNormal>::Ptr tree3 (new pcl::search::KdTree<pcl::PointNormal>);
	tree3->setInputCloud(cloud_with_normals);

	                              //This is the GreedyProjection method
    pcl::PolygonMesh triangles;

    // Set the maximum distance between connected points (maximum edge length)
	pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    gp3.setSearchRadius (0.025);
	
    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(true);
	gp3.setConsistentVertexOrdering(true);

    // Get result
	gp3.setInputCloud(cloud_with_normals);
    gp3.setSearchMethod (tree3);
    gp3.reconstruct (triangles);

	//pcl::Poisson<pcl::PointNormal> poisson; 
 //   poisson.setDepth(9); 
	////poisson.setDegree(2);
	////poisson.setConfidence(false);
	////poisson.setIsoDivide(8);
	////poisson.setManifold(false);
	////poisson.setOutputPolygons(false);
	////poisson.setSamplesPerNode(15.0);
	////poisson.setScale(1.25);
	////poisson.setSolverDivide(8);

 //   poisson.setInputCloud(cloud_with_normals); 

    pcl::io::saveVTKFile ("tsdf_cloud6_mesh.vtk", triangles);
	std::cout << "test again" << std::endl;

	//********************mesh smoothing************************
	//pcl::PolygonMesh::Ptr meshIn(new pcl::PolygonMesh);
	//*meshIn = triangles;
	//pcl::PolygonMesh smoothedMesh;
	//pcl::MeshSmoothingLaplacianVTK vtk;
	//vtk.setInputMesh(meshIn);
	//vtk.setNumIter(20000);
	//vtk.setConvergence(0.0001);
	//vtk.setRelaxationFactor(0.0001);
	//vtk.setFeatureEdgeSmoothing(true);
	//vtk.setFeatureAngle(M_PI / 5);
	//vtk.setBoundarySmoothing(true);
	//vtk.process(smoothedMesh);

  //pcl::PolygonMesh triangles;
  //pcl::io::loadPolygonFilePLY("tsdf_cloud6.ply", triangles);

	//// ################# Texture Mapping #############################

	pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(triangles.cloud, *mesh_cloud);

	// TextureMesh object
	pcl::TextureMesh mesh;
	mesh.cloud = triangles.cloud;

	// push faces into the textureMesh object
	std::vector<pcl::Vertices> polygon_1;
	polygon_1.resize(triangles.polygons.size());
	for (size_t i = 0; i < triangles.polygons.size(); ++i)
	{
		polygon_1[i] = triangles.polygons[i];
	}
	mesh.tex_polygons.push_back(polygon_1);
	PCL_INFO("\tInput mesh contains %d faces and %d vertices\n", mesh.tex_polygons[0].size(), mesh_cloud->points.size());
	PCL_INFO("...Done.\n");

	pcl::texture_mapping::CameraVector my_cams;

	// camera
	pcl::TextureMapping<pcl::PointXYZ>::Camera cam1;
	

	// translation
	cam1.pose(0, 3) = -0.17;
	cam1.pose(1, 3) = -0.40;
	cam1.pose(2, 3) = -1.05;
	//cam1.pose.translation() << 1.5, 1.5, 0.0;

	// rotation
	Eigen::Matrix3f cam1Rotation = Eigen::Matrix3f::Identity();
	cam1Rotation(0, 0) = 1;
	cam1Rotation(0, 1) = 0.0215;
	cam1Rotation(0, 2) = 0;
	cam1Rotation(1, 0) = 0;
	cam1Rotation(1, 1) = 0.9877;
	cam1Rotation(1, 2) = 0.1546;
	cam1Rotation(2, 0) = 0;
	cam1Rotation(2, 1) = -0.1546;
	cam1Rotation(2, 2) = 0.9877;

	cam1.pose(0, 0) = cam1Rotation(0, 0);
	cam1.pose(0, 1) = cam1Rotation(0, 1);
	cam1.pose(0, 2) = cam1Rotation(0, 2);
	cam1.pose(1, 0) = cam1Rotation(1, 0);
	cam1.pose(1, 1) = cam1Rotation(1, 1);
	cam1.pose(1, 2) = cam1Rotation(1, 2);
	cam1.pose(2, 0) = cam1Rotation(2, 0);
	cam1.pose(2, 1) = cam1Rotation(2, 1);
	cam1.pose(2, 2) = cam1Rotation(2, 2);
	
	// scale
	cam1.pose(3, 0) = 0.0;
	cam1.pose(3, 1) = 0.0;
	cam1.pose(3, 2) = 0.0;
	cam1.pose(3, 3) = 1.0;

	// intrinsic parameters
	cam1.focal_length = 367.221;
	cam1.center_w = 252.952;
	cam1.center_h = 208.622;
	cam1.width = 512;
	cam1.height = 424;
	cam1.texture_file = "test_tsdf_data_4/022718143547_registered_1_1.jpg";

	my_cams.push_back(cam1);

	pcl::TextureMapping<pcl::PointXYZ>::Camera cam2;

	// translation
	cam2.pose(0, 3) = 1.128942737114943e+03 / 1000.0f;
	cam2.pose(1, 3) = -1.432967440937596e+02 / 1000.0f;
	cam2.pose(2, 3) = 7.893658796324055e+02 / 1000.0f;
	cam2.pose.translation() = cam1.pose.rotation()* cam2.pose.translation() + cam1.pose.translation();
	// rotation
	cam2.pose(0, 0) = 0.134495157429655;
	cam2.pose(0, 1) = 0.080498653293076;
	cam2.pose(0, 2) = -0.987639113971279;
	cam2.pose(1, 0) = -0.144135633421401;
	cam2.pose(1, 1) = 0.987683785283996;
	cam2.pose(1, 2) = 0.060874128045401;
	cam2.pose(2, 0) = 0.980375423909730;
	cam2.pose(2, 1) = 0.134166713849142;
	cam2.pose(2, 2) = 0.144441410574456;
	cam2.pose.linear() = cam1.pose.rotation()*cam2.pose.rotation();
	// scale
	cam2.pose(3, 0) = 0.0;
	cam2.pose(3, 1) = 0.0;
	cam2.pose(3, 2) = 0.0;
	cam2.pose(3, 3) = 1.0;
	cam2.focal_length = 366.507;
	cam2.center_w = 259.864;
	cam2.center_h = 206.676;
	cam2.width = 512;
	cam2.height = 424;
	cam2.texture_file = "test_tsdf_data_4/023446243547_registered_1_1.jpg";

	my_cams.push_back(cam2);

	pcl::TextureMapping<pcl::PointXYZ>::Camera cam3;

	// translation

	cam3.pose(0, 3) = 2.507968161837474e+02 / 1000.0f;
	cam3.pose(1, 3) = -3.275127035049086e+02 / 1000.0f;
	cam3.pose(2, 3) = 2.057818523590235e+03 / 1000.0f;
	cam3.pose.translation() = cam1.pose.rotation()* cam3.pose.translation() + cam1.pose.translation();
	// rotation
	
	cam3.pose(0, 0) = -0.993943048364607;
	cam3.pose(0, 1) = 0.030325927333675;
	cam3.pose(0, 2) = -0.105629327078354;
	cam3.pose(1, 0) = 0.004398346757531;
	cam3.pose(1, 1) = 0.971379193082247;
	cam3.pose(1, 2) = 0.237493405787788;
	cam3.pose(2, 0) = 0.109808338269340;
	cam3.pose(2, 1) = 0.235590325306951;
	cam3.pose(2, 2) = -0.965628980234278;
	cam3.pose.linear() = cam1.pose.rotation()*cam3.pose.rotation();
	// scale
	cam3.pose(3, 0) = 0.0;
	cam3.pose(3, 1) = 0.0;
	cam3.pose(3, 2) = 0.0;
	cam3.pose(3, 3) = 1.0;
	// intrinsic parameters
	cam3.focal_length = 366.996;
	cam3.center_w = 256.466;
	cam3.center_h = 209.012;
	cam3.width = 512;
	cam3.height = 424;
	cam3.texture_file = "test_tsdf_data_4/001489661447_registered_1_1.jpg";

	my_cams.push_back(cam3);

	pcl::TextureMapping<pcl::PointXYZ>::Camera cam4;

	// translation
	cam4.pose(0, 3) = -8.272323915033728e+02 / 1000.0f;
	cam4.pose(1, 3) = -1.737204056866098e+02 / 1000.0f;
	cam4.pose(2, 3) = 1.256884341974619e+03 / 1000.0f;
	cam4.pose.translation() = cam1.pose.rotation()* cam4.pose.translation() + cam1.pose.translation();

	cam4.pose(0, 0) = -0.073896009032637;
	cam4.pose(0, 1) = -0.155250763238798;
	cam4.pose(0, 2) = 0.985107395344700;
	cam4.pose(1, 0) = 0.158951733970376;
	cam4.pose(1, 1) = 0.973346494691277;
	cam4.pose(1, 2) = 0.165320741408982;
	cam4.pose(2, 0) = -0.984517001436166;
	cam4.pose(2, 1) = 0.168801071637521;
	cam4.pose(2, 2) = -0.047249043346561;
	cam4.pose.linear() = cam1.pose.rotation()*cam4.pose.rotation();
	// scale
	cam4.pose(3, 0) = 0.0;
	cam4.pose(3, 1) = 0.0;
	cam4.pose(3, 2) = 0.0;
	cam4.pose(3, 3) = 1.0;
	// intrinsic parameters
	cam4.focal_length = 366.604;
	cam4.center_w = 257.112;
	cam4.center_h = 198.123;
	cam4.width = 512;
	cam4.height = 424;
	cam4.texture_file = "test_tsdf_data_4/001496161447_registered_1_1.jpg";

	my_cams.push_back(cam4);

    PCL_INFO("\tLoaded the texture.\n");
	PCL_INFO("...Done.\n");

	// Create materials for each texture (and one extra for occluded faces)
	mesh.tex_materials.resize(my_cams.size() + 1);
	for (int i = 0; i <= my_cams.size(); ++i)
	{
		pcl::TexMaterial mesh_material;
		mesh_material.tex_Ka.r = 0.2f;
		mesh_material.tex_Ka.g = 0.2f;
		mesh_material.tex_Ka.b = 0.2f;

		mesh_material.tex_Kd.r = 0.8f;
		mesh_material.tex_Kd.g = 0.8f;
		mesh_material.tex_Kd.b = 0.8f;

		mesh_material.tex_Ks.r = 1.0f;
		mesh_material.tex_Ks.g = 1.0f;
		mesh_material.tex_Ks.b = 1.0f;

		mesh_material.tex_d = 1.0f;
		mesh_material.tex_Ns = 75.0f;
		mesh_material.tex_illum = 2;

		std::stringstream tex_name;
		tex_name << "material_" << i;
		tex_name >> mesh_material.tex_name;

		if (i < my_cams.size())
			mesh_material.tex_file = my_cams[i].texture_file;
		else
			mesh_material.tex_file = "occluded.jpg";

		mesh.tex_materials[i] = mesh_material;
	}
	// Sort faces
	PCL_INFO("\nSorting faces by cameras...\n");
	pcl::TextureMapping<pcl::PointXYZ> tm; // TextureMapping object that will perform the sort
	tm.textureMeshwithMultipleCameras(mesh, my_cams);

	// compute normal for the mesh
	PCL_INFO("\nEstimating normals...\n");
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> mesh_ne;
	pcl::PointCloud<pcl::Normal>::Ptr mesh_normals(new pcl::PointCloud<pcl::Normal>);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr mesh_tree(new pcl::search::KdTree<pcl::PointXYZ>);
	mesh_tree->setInputCloud(mesh_cloud);
	mesh_ne.setInputCloud(mesh_cloud);
	mesh_ne.setSearchMethod(mesh_tree);
	mesh_ne.setKSearch(20);
	mesh_ne.compute(*mesh_normals);

	// Concatenate XYZ and normal fields
	pcl::PointCloud<pcl::PointNormal>::Ptr mesh_cloud_with_normals(new pcl::PointCloud<pcl::PointNormal>);
	pcl::concatenateFields(*mesh_cloud, *mesh_normals, *mesh_cloud_with_normals);
	PCL_INFO("...Done.\n");

	pcl::toPCLPointCloud2(*mesh_cloud_with_normals, mesh.cloud);

	PCL_INFO("\nSaving mesh to textured_mesh.obj\n");

	saveOBJFile("mesh_nomls1.obj", mesh, 5);

    return (0);
}
