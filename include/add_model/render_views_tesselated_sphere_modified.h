/*
 * render_views_tesselated_sphere.h
 *
 *  Created on: Dec 23, 2011
 *      Author: aitor
 *
 *  Modified on: July - Dec, 2017
 *		By: Andreas
 */

#ifndef RENDER_VIEWS_TESSELATED_SPHERE_MODIFIED_H_
#define RENDER_VIEWS_TESSELATED_SPHERE_MODIFIED_H_

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <pcl/common/common.h>
#include <boost/function.hpp>

#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <vtkCellData.h>
#include <vtkWorldPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkPlatonicSolidSource.h>
#include <vtkLoopSubdivisionFilter.h>
#include <vtkTriangle.h>
#include <vtkTransform.h>
#if VTK_MAJOR_VERSION>=6 || (VTK_MAJOR_VERSION==5 && VTK_MINOR_VERSION>4)
#include <vtkHardwareSelector.h>
#include <vtkSelectionNode.h>
#else 
#include <vtkVisibleCellSelector.h>
#endif
#include <vtkSelection.h>
#include <vtkCellArray.h>
#include <vtkTransformFilter.h>
#include <vtkCamera.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkPolyDataMapper.h>
#include <vtkPointPicker.h>
  
/** \brief @b Class to render synthetic views of a 3D mesh using a tesselated sphere
 * NOTE: This class should replace renderViewTesselatedSphere from pcl::visualization.
 * Some extensions are planned in the near future to this class like removal of duplicated views for
 * symmetrical objects, generation of RGB synthetic clouds when RGB available on mesh, etc.
 * \author Aitor Aldoma
 * \ingroup apps
 */
class Render_Views_Tesselated_Sphere_Modified
{
private:
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > poses_; // MODIFIED CODE
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > original_poses_;
  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > cam_positions_;
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> generated_views_;
  std::vector<float> entropies_;
  int resolution_;
  int tessellation_level_;
  bool use_vertices_;
  float view_angle_;
  float radius_sphere_;
  bool compute_entropy_;
  vtkSmartPointer<vtkPolyData> polydata_;
  bool gen_organized_;
  boost::function<bool
  (const Eigen::Vector3f &)> campos_constraints_func_;
  double scale_factor_;
  double object_radius_;
  bool use_delay_;

  struct camPosConstraintsAllTrue
  {
    bool
    operator() (const Eigen::Vector3f & /*pos*/) const
    {
      return true;
    }
    ;
  };

public:
  Render_Views_Tesselated_Sphere_Modified ()
  {
    use_vertices_ = false;
    compute_entropy_ = false;
    gen_organized_ = false;
    campos_constraints_func_ = camPosConstraintsAllTrue ();
  }

  void
  setCamPosConstraints (boost::function<bool (const Eigen::Vector3f &)> & bb)
  {
    campos_constraints_func_ = bb;
  }

  /* \brief Indicates wether to generate organized or unorganized data
   * \param b organized/unorganized
   */
  void
  setGenOrganized (bool b)
  {
    gen_organized_ = b;
  }

  /* \brief Sets the size of the render window
   * \param res resolution size
   */
  void
  setResolution (int res)
  {
    resolution_ = res;
  }

  /* \brief Wether to use the vertices or triangle centers of the tesselated sphere
   * \param use true indicates to use vertices, false triangle centers
   */

  void
  setUseVertices (bool use)
  {
    use_vertices_ = use;
  }

  /* \brief Radius of the sphere where the virtual camera will be placed
   * \param use true indicates to use vertices, false triangle centers
   */
  void
  setRadiusSphere (float radius)
  {
    radius_sphere_ = radius;
  }

  /* \brief Wether to compute the entropies (level of occlusions for each view)
   * \param compute true to compute entropies, false otherwise
   */
  void
  setComputeEntropies (bool compute)
  {
    compute_entropy_ = compute;
  }

  /* \brief How many times the icosahedron should be tesselated. Results in more or less camera positions and generated views.
   * \param level amount of tessellation
   */
  void
  setTessellationLevel (int level)
  {
    tessellation_level_ = level;
  }
  
  //
  // MODIFIED CODE BEGINS
  //

  void
  setScaleFactor (double scale_factor)
  {
  	scale_factor_ = scale_factor;
  }
  
  void
  setObjectRadius (double object_radius)
  {
  	object_radius_ = object_radius;
  }

  //
  // MODIFIED CODE ENDS
  //

  /* \brief Sets the view angle of the virtual camera
   * \param angle view angle in degrees
   */
  void
  setViewAngle (float angle)
  {
    view_angle_ = angle;
  }

  /* \brief adds the mesh to be used as a vtkPolyData
   * \param polydata vtkPolyData object
   */
  void
  addModelFromPolyData (vtkSmartPointer<vtkPolyData> &polydata)
  {
    polydata_ = polydata;
  }

  /* \brief performs the rendering and stores the generated information
   */
  void
  generateViews ();

  //
  // MODIFIED CODE BEGINS
  //

  void
  getPoses (std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > & poses)
  {
    poses = poses_;
  }
  
  void
  getCamPositions (std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d> > & cam_positions)
  {
  	cam_positions = cam_positions_;
  }
  
  void
  getOriginalPoses (std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d> > & poses)
  {
    poses = original_poses_;
  }

  //
  // MODIFIED CODE ENDS
  //

  /* \brief Get the generated views
   * \param views generated pointclouds in camera coordinates
   */
  void
  getViews (std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> & views)
  {
    views = generated_views_;
  }

  /* \brief Get the entropies (level of occlusions) for the views
   * \param entropies level of occlusions
   */
  void
  getEntropies (std::vector<float> & entropies)
  {
    entropies = entropies_;
  }
  
  //
  // MODIFIED CODE BEGINS
  //
  
  /**
    Set if program should use a small delay before rendering the first view. 
    @param use_delay True if use small delay, false otherwise
  */
  void
  setUseDelay(bool use_delay)
  {
  	use_delay_ = use_delay;
  }
  
  //
  // MODIFIED CODE ENDS
  //
};

#endif /* RENDER_VIEWS_TESSELATED_SPHERE_MODIFIED_H_ */
