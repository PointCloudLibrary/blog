OBJ_IO Source Code
=======================================
   
.. _obj_io:

OBJ_IO .H FILE
-------------------------------------------

.. code-block:: cpp

	/*
	 * obj_io.h
	 *
	 *  Created on: Jun 7, 2011
	 *      Author: ktran
	 */

	#ifndef OBJ_IO_H_
	#define OBJ_IO_H_
	#include <pcl/pcl_macros.h>
	#include <pcl/PolygonMesh.h>
	#include <pcl/surface/texture.h>
	namespace pcl
	{
	  namespace io
	  {
	    /** \brief Saves a PolygonMesh in ascii OBJ format.
	      * \param file_name the name of the file to write to disk
	      * \param triangles the polygonal mesh to save
	      * \param precision the output ASCII precision
	      * \ingroup io
	      */
	    PCL_EXPORTS int
	    saveOBJFile (const std::string &file_name,
	    	    const pcl::TextureMesh &tex_Mesh, unsigned precision = 5);
	  }
	}

	#endif /* OBJ_IO_H_ */


OBJ_IO .CPP FILE
-------------------------------------------

.. code-block:: cpp

	/*
	 * obj_io.cpp
	 *
	 *  Created on: Jun 7, 2011
	 *      Author: ktran
	 */
	#include <pcl/io/obj_io.h>
	#include <fstream>
	#include <iostream>
	#include <pcl/io/io.h>

	//////////////////////////////////////////////////////////////////////////////////////////////
	int
	pcl::io::saveOBJFile (const std::string &file_name,
		    const pcl::TextureMesh &tex_Mesh, unsigned precision)
	{
	  if (tex_Mesh.cloud.data.empty ())
	  {
	    PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no data!\n");
	    return (-1);
	  }
	  // Open file
	  std::ofstream fs;
	  fs.precision (precision);
	  fs.open (file_name.c_str ());

	  // Define material file
	  std::string mtl_file_name = file_name.substr(0, file_name.find_last_of("."))+".mtl";

	  /* Write 3D information */
	  // number of points
	  int nr_points  = tex_Mesh.cloud.width * tex_Mesh.cloud.height;
	  int point_size = tex_Mesh.cloud.data.size () / nr_points;
	  // number of facets
	  int nr_faces = tex_Mesh.tex_polygons.size();
	  // number of normal vectors
	  int nr_normals = tex_Mesh.tex_normals.size();
	  // number of normal vectors
	  int nr_texcoordinates = tex_Mesh.tex_coordinates.size();

	  // Write the header information
	  fs << "####" << std::endl;
	  fs << "# OBJ dataFile simple version. File name: " << file_name << std::endl;
	  fs << "# Vertices: " << nr_points << std::endl;
	  fs << "# Faces: " <<nr_faces << std::endl;
	  fs << "# Material information:" << std::endl;
	  fs <<"mtllib " <<mtl_file_name << std::endl;
	  fs << "####" << std::endl;
	  fs << "g Group001" << std::endl;

	  // Write vertex coordinates
	  fs << "# Vertices" << std::endl;

	  for (int i = 0; i < nr_points; ++i)
	  {
	    int xyz = 0;
	    // "v" just be written one
	    bool v_written = false;
	    for (size_t d = 0; d < tex_Mesh.cloud.fields.size (); ++d)
	    {
	      int count = tex_Mesh.cloud.fields[d].count;
	      if (count == 0)
		count = 1;          // we simply cannot tolerate 0 counts (coming from older converter code)
	      int c = 0;
	      // adding vertex
	      if ((tex_Mesh.cloud.fields[d].datatype == sensor_msgs::PointField::FLOAT32) && (
	    		  tex_Mesh.cloud.fields[d].name == "x" ||
	    		  tex_Mesh.cloud.fields[d].name == "y" ||
	    		  tex_Mesh.cloud.fields[d].name == "z"))
	      {
	    	if(!v_written)
	    	{
	    		 // write vertices beginning with v
	    		fs << "v ";
	    		v_written = true;
	    	}
		float value;
		memcpy (&value, &tex_Mesh.cloud.data[i * point_size + tex_Mesh.cloud.fields[d].offset + c * sizeof (float)], sizeof (float));
		fs << value;
		if (++xyz == 3)
		  break;
	      }
	      fs << " ";
	    }
	    if (xyz != 3)
	    {
	      PCL_ERROR ("[pcl::io::saveOBJFile] Input point cloud has no XYZ data!\n");
	      return (-2);
	    }
	    fs << std::endl;
	  }
	  fs << "# "<< nr_points <<" vertices" << std::endl;

	  // Write normal vector with "vn" (now we don't have it)
	  fs << "# "<< nr_normals <<" normal" << std::endl;
	  for (size_t i = 0; i < tex_Mesh.tex_normals.size(); ++i){
		fs << "vn ";
		fs << tex_Mesh.tex_normals[i].normal_x;
		fs << " " ;
		fs << tex_Mesh.tex_normals[i].normal_y;
		fs << " ";
		fs << tex_Mesh.tex_normals[i].normal_z << std::endl;
	  }
	    // Write vertex texture with "vt" (adding latter)
	  fs << "# "<< nr_texcoordinates <<" vertex textures" << std::endl;
	  for (size_t i = 0; i < tex_Mesh.tex_coordinates.size(); ++i){
		fs << "vt ";
		fs <<  tex_Mesh.tex_coordinates[i].x << " " << tex_Mesh.tex_coordinates[i].y << std::endl;
	  }

	  // Specify the material will be used
	  fs << "# The material will be used" << std::endl;
	  fs << "usemtl " <<  tex_Mesh.tex_material.tex_name << std::endl;
	  // Write faces with "f"
	  fs << "# Faces" << std::endl;
	  for (size_t i = 0; i < tex_Mesh.tex_polygons.size (); ++i)
	  {
		fs << "f ";
	    size_t j = 0;
	    for (j = 0; j < tex_Mesh.tex_polygons[i].vertices.size () - 1; ++j)
	      fs << tex_Mesh.tex_polygons[i].vertices[j] +1 <<"/" << tex_Mesh.tex_polygons[i].vertices[j] +1 << " "; // vertex index in obj file format starting with 1
	    fs << tex_Mesh.tex_polygons[i].vertices[j]+1 <<"/" << tex_Mesh.tex_polygons[i].vertices[j] +1 << std::endl;
	  }
	  fs << "# "<< nr_faces <<" faces" << std::endl;
	  fs << "# End of File";

	  // Close obj file
	  fs.close ();

	  /* Write material defination for OBJ file*/
	  // Open file

	  std::ofstream m_fs;
	  m_fs.precision (precision);
	  m_fs.open (mtl_file_name.c_str ());

	  // default
	  m_fs << "#" << std::endl;
	  m_fs << "# Wavefront material file" << std::endl;
	  m_fs << "#" << std::endl;

	  m_fs << "newmtl " << tex_Mesh.tex_material.tex_name << std::endl;
	  m_fs << "Ka "<< tex_Mesh.tex_material.tex_Ka.r << " " << tex_Mesh.tex_material.tex_Ka.g << " " << tex_Mesh.tex_material.tex_Ka.b << std::endl; // defines the ambient color of the material to be (r,g,b).
	  m_fs << "Kd "<< tex_Mesh.tex_material.tex_Kd.r << " " << tex_Mesh.tex_material.tex_Kd.g << " " << tex_Mesh.tex_material.tex_Kd.b << std::endl; // defines the diffuse color of the material to be (r,g,b).
	  m_fs << "Ks "<< tex_Mesh.tex_material.tex_Ks.r << " " << tex_Mesh.tex_material.tex_Ks.g << " " << tex_Mesh.tex_material.tex_Ks.b << std::endl; // defines the specular color of the material to be (r,g,b). This color shows up in highlights.
	  m_fs << "d "<< tex_Mesh.tex_material.tex_d << std::endl; // defines the transparency of the material to be alpha.
	  m_fs << "Ns "<< tex_Mesh.tex_material.tex_Ns  << std::endl; // defines the shininess of the material to be s.
	  m_fs << "illum "<< tex_Mesh.tex_material.tex_illum << std::endl; // denotes the illumination model used by the material.
	  	  	  	  	  	  // illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
	  	  	  	  	  	  // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
	  m_fs << "map_Kd " << tex_Mesh.tex_material.tex_file << std::endl;
	  m_fs.close();
	  return (0);
	}

TEXTURE .H
-------------------------------------------

.. code-block:: cpp

	/*
	 * texture.h
	 *
	 *  Created on: Jun 10, 2011
	 *      Author: ktran
	 */

	#ifndef TEXTURE_H_
	#define TEXTURE_H_
	#include <string>
	#include <vector>
	#include <ostream>

	// Include the correct Header path here
	#include <pcl/PolygonMesh.h>
	#include "std_msgs/Header.h"
	#include "sensor_msgs/PointCloud2.h"
	#include "pcl/Vertices.h"
	#include "pcl/point_types.h"

	namespace pcl
	{
	  struct RGB{
	    float r;
		float g;
		float b;
	  }; //RGB

	  struct TexMaterial
	  {
		  std::string		tex_name; 	// texture name
	      std::string  		tex_file; 	// texture file
	      pcl::RGB  		tex_Ka; 	// defines the ambient color of the material to be (r,g,b).
	      pcl::RGB  		tex_Kd; 	// defines the diffuse color of the material to be (r,g,b).
	      pcl::RGB  		tex_Ks; 	// defines the specular color of the material to be (r,g,b). This color shows up in highlights.
	      float 			tex_d; 		// defines the transparency of the material to be alpha.
	      float 			tex_Ns;  	// defines the shininess of the material to be s.
	      int				tex_illum;	// denotes the illumination model used by the material.
	  										  	// illum = 1 indicates a flat material with no specular highlights, so the value of Ks is not used.
	  										    // illum = 2 denotes the presence of specular highlights, and so a specification for Ks is required.
	  }; // TexMaterial
	  struct TextureMesh
	  {
		TextureMesh () : header (), cloud (), tex_polygons ()
		{}

		::std_msgs::Header  			header;

		::sensor_msgs::PointCloud2 		cloud;

		std::vector< ::pcl::Vertices>  	tex_polygons; // polygon which is mapped with specific texture defined in TexMaterial
		std::vector< ::pcl::Normal> 	tex_normals; // normal vertices
		std::vector< ::pcl::PointXY>  	tex_coordinates; // UV coordinates
		TexMaterial						tex_material; // define texture material
		public:
		  typedef boost::shared_ptr< ::pcl::PolygonMesh> Ptr;
		  typedef boost::shared_ptr< ::pcl::PolygonMesh const> ConstPtr;

	  }; // struct TextureMesh

	  typedef boost::shared_ptr< ::pcl::TextureMesh> TextureMeshPtr;
	  typedef boost::shared_ptr< ::pcl::TextureMesh const> TextureMeshConstPtr;

	} // namespace pcl

	#endif /* TEXTURE_H_ */



TEXTURE MAPPING TEST FILE
-------------------------------------------

.. code-block:: cpp

	TEST (PCL, TextureMapping)
	{
	  // Init objects
	  PolygonMesh triangles;
	  GreedyProjectionTriangulation<PointNormal> gp3;

	  // Set parameters
	  gp3.setInputCloud (cloud_with_normals);
	  gp3.setSearchMethod (tree2);
	  gp3.setSearchRadius (0.025);
	  gp3.setMu (2.5);
	  gp3.setMaximumNearestNeighbors (100);
	  gp3.setMaximumSurfaceAgle(M_PI/4); // 45 degrees
	  gp3.setMinimumAngle(M_PI/18); // 10 degrees
	  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
	  gp3.setNormalConsistency(false);

	  // Reconstruct to get a triangle mesh
	  gp3.reconstruct (triangles);

	  // Texture into mesh
	  TextureMesh texMesh; 

	  texMesh.tex_polygons = triangles.polygons;
	  texMesh.header = triangles.header;
	  texMesh.cloud = triangles.cloud;


	  // 1ST MAPPING: UV mapping (similar to OpenGL per-vertex textures) - Texture is being mapped in the Y plane

	  int nr_points  = texMesh.cloud.width * texMesh.cloud.height;
	  int point_size = texMesh.cloud.data.size () / nr_points;

	  float x_lowest = 100000;
	  float x_highest = 0 ;
	  float y_lowest = 100000;
	  float y_highest = 0 ;
	  float z_lowest = 100000;
	  float z_highest = 0;
	  float x_, y_, z_;
	  for (int i =0; i < nr_points; ++i){
		memcpy (&x_, &texMesh.cloud.data[i * point_size + texMesh.cloud.fields[0].offset + 0 * sizeof (float) ], sizeof (float));
		memcpy (&y_, &texMesh.cloud.data[i * point_size + texMesh.cloud.fields[1].offset + 0 * sizeof (float)], sizeof (float));
		memcpy (&z_, &texMesh.cloud.data[i * point_size + texMesh.cloud.fields[2].offset + 0 * sizeof (float)], sizeof (float));
		// x
		if (x_ <= x_lowest) x_lowest = x_;
		if (x_ > x_lowest) x_highest = x_;

		// y
		if (y_ <= y_lowest) y_lowest = y_;
	    	if (y_ > y_lowest) y_highest = y_;

		// z
		if (z_ <= z_lowest) z_lowest = z_;
		if (z_ > z_lowest) z_highest = z_;
	  }
	  // x
	  float x_range = (x_lowest - x_highest)*-1;
	  float x_offset = 0 - x_lowest;
	  // x
	  float y_range = (y_lowest - y_highest)*-1;
	  float y_offset = 0 - y_lowest;
	  // z
	  float z_range = (z_lowest - z_highest)*-1;
	  float z_offset = 0 - z_lowest;

	  std::vector<PointXY> tmp_coordinates;
	  PointXY tmp_VT;
	  for (int i =0; i < nr_points; ++i){
		memcpy (&x_, &triangles.cloud.data[i * point_size + triangles.cloud.fields[0].offset + 0 * sizeof (float)], sizeof (float));
		memcpy (&y_, &triangles.cloud.data[i * point_size + triangles.cloud.fields[1].offset + 0 * sizeof (float)], sizeof (float));
		memcpy (&z_, &triangles.cloud.data[i * point_size + triangles.cloud.fields[2].offset + 0 * sizeof (float)], sizeof (float));
		// calculate uv coordinates
		tmp_VT.x = (x_ + x_offset)/x_range;
		tmp_VT.y = (z_ + z_offset)/z_range;
		tmp_coordinates.push_back(tmp_VT);
	  }
	  texMesh.tex_coordinates = tmp_coordinates;

	  // material
	  texMesh.tex_material.tex_name = "material_0";
	  texMesh.tex_material.tex_file = "bun0.png";
	  texMesh.tex_material.tex_Ka.r = 0.2f;
	  texMesh.tex_material.tex_Ka.g = 0.2f;
	  texMesh.tex_material.tex_Ka.b = 0.2f;

	  texMesh.tex_material.tex_Kd.r = 0.8f;
	  texMesh.tex_material.tex_Kd.g = 0.8f;
	  texMesh.tex_material.tex_Kd.b = 0.8f;

	  texMesh.tex_material.tex_Ks.r = 1.0f;
	  texMesh.tex_material.tex_Ks.g = 1.0f;
	  texMesh.tex_material.tex_Ks.b = 1.0f;
	  texMesh.tex_material.tex_d = 1.0f;
	  texMesh.tex_material.tex_Ns = 0.0f;
	  texMesh.tex_material.tex_illum = 2;

	  saveOBJFile ("./bun0-gp3.obj", texMesh);

	}

