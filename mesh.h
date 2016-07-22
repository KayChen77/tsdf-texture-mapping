// created by czr. 08.06.2016

#include <iostream>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Cholesky>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/vtk_smoothing/vtk.h>
#include <pcl/surface/vtk_smoothing/vtk_mesh_smoothing_laplacian.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/texture_mapping.h>
#include <pcl/surface/concave_hull.h>



// compute the average distance between points
double
computeCloudResolution(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<pcl::PointXYZ> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->points.size(); ++i)
	{
		if (!pcl_isfinite(cloud->points[i].x) || !pcl_isfinite(cloud->points[i].y) || !pcl_isfinite(cloud->points[i].z))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

/** \brief Save a textureMesh object to obj file */
int
saveOBJFile(const std::string &file_name,
const pcl::TextureMesh &tex_mesh, unsigned precision)
{
	if (tex_mesh.cloud.data.empty())
	{
		PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
		return (-1);
	}

	// Open file
	std::ofstream fs;
	fs.precision(precision);
	fs.open(file_name.c_str());

	// Define material file
	std::string mtl_file_name = file_name.substr(0, file_name.find_last_of(".")) + ".mtl";
	// Strip path for "mtllib" command
	std::string mtl_file_name_nopath = mtl_file_name;
	mtl_file_name_nopath.erase(0, mtl_file_name.find_last_of('/') + 1);

	/* Write 3D information */
	// number of points
	int nr_points = tex_mesh.cloud.width * tex_mesh.cloud.height;
	int point_size = tex_mesh.cloud.data.size() / nr_points;

	// mesh size
	int nr_meshes = tex_mesh.tex_polygons.size();
	// number of faces for header
	int nr_faces = 0;
	for (int m = 0; m < nr_meshes; ++m)
		nr_faces += tex_mesh.tex_polygons[m].size();

	// Write the header information
	fs << "####" << std::endl;
	fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
	fs << "# Vertices: " << nr_points << std::endl;
	fs << "# Faces: " << nr_faces << std::endl;
	fs << "# Material information:" << std::endl;
	fs << "mtllib " << mtl_file_name_nopath << std::endl;
	fs << "####" << std::endl;

	// Write vertex coordinates
	fs << "# Vertices" << std::endl;
	for (int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		// "v" just be written one
		bool v_written = false;
		for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d)
		{
			int count = tex_mesh.cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			// adding vertex
			if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
				tex_mesh.cloud.fields[d].name == "x" ||
				tex_mesh.cloud.fields[d].name == "y" ||
				tex_mesh.cloud.fields[d].name == "z"))
			{
				if (!v_written)
				{
					// write vertices beginning with v
					fs << "v ";
					v_written = true;
				}
				float value;
				memcpy(&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
				fs << value;
				if (++xyz == 3)
					break;
				fs << " ";
			}
		}
		if (xyz != 3)
		{
			PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
			return (-2);
		}
		fs << std::endl;
	}
	fs << "# " << nr_points << " vertices" << std::endl;

	// Write vertex normals
	for (int i = 0; i < nr_points; ++i)
	{
		int xyz = 0;
		// "vn" just be written one
		bool v_written = false;
		for (size_t d = 0; d < tex_mesh.cloud.fields.size(); ++d)
		{
			int count = tex_mesh.cloud.fields[d].count;
			if (count == 0)
				count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
			int c = 0;
			// adding vertex
			if ((tex_mesh.cloud.fields[d].datatype == pcl::PCLPointField::FLOAT32) && (
				tex_mesh.cloud.fields[d].name == "normal_x" ||
				tex_mesh.cloud.fields[d].name == "normal_y" ||
				tex_mesh.cloud.fields[d].name == "normal_z"))
			{
				if (!v_written)
				{
					// write vertices beginning with vn
					fs << "vn ";
					v_written = true;
				}
				float value;
				memcpy(&value, &tex_mesh.cloud.data[i * point_size + tex_mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
				fs << value;
				if (++xyz == 3)
					break;
				fs << " ";
			}
		}
		if (xyz != 3)
		{
			PCL_ERROR("[pcl::io::saveOBJFile] Input point cloud has no normals!\n");
			return (-2);
		}
		fs << std::endl;
	}
	// Write vertex texture with "vt" (adding latter)

	for (int m = 0; m < nr_meshes; ++m)
	{
		if (tex_mesh.tex_coordinates.size() == 0)
			continue;

		PCL_INFO("%d vertex textures in submesh %d\n", tex_mesh.tex_coordinates[m].size(), m);
		fs << "# " << tex_mesh.tex_coordinates[m].size() << " vertex textures in submesh " << m << std::endl;
		for (size_t i = 0; i < tex_mesh.tex_coordinates[m].size(); ++i)
		{
			fs << "vt ";
			fs << tex_mesh.tex_coordinates[m][i][0] << " " << tex_mesh.tex_coordinates[m][i][1] << std::endl;
		}
	}

	int f_idx = 0;

	// int idx_vt =0;
	PCL_INFO("Writting faces...\n");
	for (int m = 0; m < nr_meshes; ++m)
	{
		if (m > 0)
			f_idx += tex_mesh.tex_polygons[m - 1].size();

		if (tex_mesh.tex_materials.size() != 0)
		{
			fs << "# The material will be used for mesh " << m << std::endl;
			//TODO pbl here with multi texture and unseen faces
			fs << "usemtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
			fs << "# Faces" << std::endl;
		}
		for (size_t i = 0; i < tex_mesh.tex_polygons[m].size(); ++i)
		{
			// Write faces with "f"
			fs << "f";
			size_t j = 0;
			// There's one UV per vertex per face, i.e., the same vertex can have
			// different UV depending on the face.
			for (j = 0; j < tex_mesh.tex_polygons[m][i].vertices.size(); ++j)
			{
				unsigned int idx = tex_mesh.tex_polygons[m][i].vertices[j] + 1;
				fs << " " << idx
					<< "/" << 3 * (i + f_idx) + j + 1
					<< "/" << idx; // vertex index in obj file format starting with 1
			}
			fs << std::endl;
		}
		PCL_INFO("%d faces in mesh %d \n", tex_mesh.tex_polygons[m].size(), m);
		fs << "# " << tex_mesh.tex_polygons[m].size() << " faces in mesh " << m << std::endl;
	}
	fs << "# End of File";

	// Close obj file
	PCL_INFO("Closing obj file\n");
	fs.close();

	/* Write material defination for OBJ file*/
	// Open file
	PCL_INFO("Writing material files\n");
	//dont do it if no material to write
	if (tex_mesh.tex_materials.size() == 0)
		return (0);

	std::ofstream m_fs;
	m_fs.precision(precision);
	m_fs.open(mtl_file_name.c_str());

	// default
	m_fs << "#" << std::endl;
	m_fs << "# Wavefront material file" << std::endl;
	m_fs << "#" << std::endl;
	for (int m = 0; m < nr_meshes; ++m)
	{
		m_fs << "newmtl " << tex_mesh.tex_materials[m].tex_name << std::endl;
		m_fs << "Ka " << tex_mesh.tex_materials[m].tex_Ka.r << " " << tex_mesh.tex_materials[m].tex_Ka.g << " " << tex_mesh.tex_materials[m].tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
		m_fs << "Kd " << tex_mesh.tex_materials[m].tex_Kd.r << " " << tex_mesh.tex_materials[m].tex_Kd.g << " " << tex_mesh.tex_materials[m].tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
		m_fs << "Ks " << tex_mesh.tex_materials[m].tex_Ks.r << " " << tex_mesh.tex_materials[m].tex_Ks.g << " " << tex_mesh.tex_materials[m].tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
		m_fs << "d " << tex_mesh.tex_materials[m].tex_d << std::endl; // defines the transparency of the material to be alpha.
		m_fs << "Ns " << tex_mesh.tex_materials[m].tex_Ns << std::endl; // defines the shininess of the material to be s.
		m_fs << "illum " << tex_mesh.tex_materials[m].tex_illum << std::endl; // denotes the illumination model used by the material.
		// illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
		// illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
		m_fs << "map_Kd " << tex_mesh.tex_materials[m].tex_file << std::endl;
		m_fs << "###" << std::endl;
	}
	m_fs.close();
	return (0);
}

