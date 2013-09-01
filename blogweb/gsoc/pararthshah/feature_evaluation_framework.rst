Feature Correspondence Test Source Code
=======================================
   
.. _feature_test:

FeatureCorrespondenceTest Class
-------------------------------

The file feature_test.h contains the implementation of the FeatureCorrespondenceTest class, which is the base class for performing any feature descriptor tests.
Currently I have included a sample implementation of FPFHTest, which is used for testing the FPFHEstimation class. Other test classes will follow a similar structure. 

.. code-block:: cpp
   :linenos:
   
	#include <vector>
	#include <map>
	
	#include <boost/shared_ptr.hpp>
	#include <boost/lexical_cast.hpp>
	
	#include <pcl/pcl_base.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	
	#include <pcl/kdtree/tree_types.h>
	#include <pcl/kdtree/kdtree.h>
	#include <pcl/kdtree/kdtree_flann.h>
	
	#include <pcl/features/feature.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/features/fpfh.h>
	
	#include <pcl/registration/transforms.h>
	
	#include <Eigen/Core>
	
	namespace pcl
	{
	  /** \brief FeatureCorrespondenceTest is the base class implementing the functionality for running Feature Correspondence tests.
	    *
	    * To test a Feature Descriptor algorithm, derive a separate class corresponding to that algorithm from this base class.
	    * Implement following methods:
	    * setParameters(ParameterList) Provide input parameters
	    * computeFeatures() Compute feature descriptors
	    * computeCorrespondences() Compute correspondences between source and target feature descriptors
	    */
	  template <typename PointIn>
	  class FeatureCorrespondenceTest: public PCLBase<PointIn>
	  {
	  public:
	    typedef pcl::PointCloud<PointIn> PointCloudIn;
	    typedef typename PointCloudIn::Ptr PointCloudInPtr;
	    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
	
	    typedef std::map<int, int> MapSourceTargetIndices;
	    typedef MapSourceTargetIndices* MapSourceTargetIndicesPtr;
	
	    typedef std::map <std::string, std::string> ParameterList;
	
	    typedef typename boost::shared_ptr<FeatureCorrespondenceTest<PointIn> > Ptr;
	
	  public:
	    /** \brief Empty constructor
	      */
	     FeatureCorrespondenceTest () : source_input_(), target_input_(), source_transform_(new pcl::PointCloud<PointIn>),
	                                    correspondences_(), ground_truths_(Eigen::Matrix4f::Identity ()),
	                                    search_threshold_(0.01), no_of_successes_(0), no_of_failures_(0) {}
	
	    inline void
	    setInputClouds (const PointCloudInPtr &source, const PointCloudInPtr &target)
	    {
	      source_input_ = source;
	      target_input_ = target;
	    }
	
	    inline void
	    setSearchThreshold (float threshold) { search_threshold_ = threshold; }
	
	    /** \brief Store the "ground truth" correspondences between source and target.
	      *
	      * \param ground_truths Map of source point index to corresponding target point index.
	      */
	    inline void
	    setGroundTruths (const Eigen::Matrix4f &ground_truths)
	    {
	      ground_truths_ = ground_truths;
	    }
	
	    /** \brief Calculate number of correspondences within \a search_threshold_ of respective ground truth point
	      *
	      */
	    void
	    computeResults ();
	
	    inline int
	    getSuccesses () { return no_of_successes_; }
	
	    inline int
	    getFailures () { return no_of_failures_; }
	
	    /** \brief Temporary fix until FeatureCorrespondenceTest is made a friend of the Feature Estimation class.
	      *
	      */
	    virtual std::string
	    getClassName () { return "FeatureTest"; }		//Never used
	
	    virtual void
	    setParameters (ParameterList params) {}
	
	    virtual void
	    computeFeatures () {}
	
	    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
	      *
	      */
	    virtual void
	    computeCorrespondences () {}
	
	  protected:
	
	    PointCloudInPtr source_input_;
	    PointCloudInPtr target_input_;
	    PointCloudInPtr source_transform_;
	
	    MapSourceTargetIndicesPtr correspondences_;
	
	    Eigen::Matrix4f ground_truths_;
	
	    float search_threshold_;
	
	    int no_of_successes_, no_of_failures_;
	
	  };
	
	  template <typename PointIn, typename NormalT, typename FeatureDescriptor>
	  class FPFHTest : public FeatureCorrespondenceTest<PointIn>
	  {
	  public:
	    using FeatureCorrespondenceTest<PointIn>::source_input_;
	    using FeatureCorrespondenceTest<PointIn>::target_input_;
	    using FeatureCorrespondenceTest<PointIn>::correspondences_;
	
	    typedef pcl::PointCloud<FeatureDescriptor> Features;
	    typedef typename Features::Ptr FeaturesPtr;
	    typedef typename Features::ConstPtr FeaturesConstPtr;
	
	    typedef typename pcl::KdTree<FeatureDescriptor> KdTree;
	    typedef typename pcl::KdTree<FeatureDescriptor>::Ptr KdTreePtr;
	
	
	    typedef pcl::PointCloud<NormalT> NormalIn;
	    typedef typename NormalIn::Ptr NormalInPtr;
	    typedef typename NormalIn::ConstPtr NormalInConstPtr;
	
	    typedef typename pcl::KdTreeFLANN<PointIn> KdTreePointIn;
	    typedef typename KdTreePointIn::Ptr KdTreePointInPtr;
	
	    typedef typename FeatureCorrespondenceTest<PointIn>::ParameterList ParameterList;
	    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndices MapSourceTargetIndices;
	    typedef typename FeatureCorrespondenceTest<PointIn>::MapSourceTargetIndicesPtr MapSourceTargetIndicesPtr;
	
	  public:
	    FPFHTest () : source_normals_(), target_normals_(), source_features_(),
	                  target_features_(), search_radius_(0.05), tree_()
	    {
	      FeatureCorrespondenceTest<PointIn> ();
	    }
	
	    inline void setRadiusSearch (float radius) { search_radius_ = radius; }
	
	    /** \brief Calculate surface normals of input source and target clouds.
	      *
	      */
	    void
	    computeNormals (float search_radius);
	
	    /** \brief Set parameters for feature correspondence test algorithm
	      *
	      */
	    void
	    setParameters (ParameterList &params);
	
	    /** \brief Compute the FPFH feature descriptors of source and target clouds
	      *
	      */
	    void
	    computeFeatures ();
	
	    /** \brief Calculate the nearest neighbour of each source_feature_ point in the target_feature_ cloud in n-D feature space
	      *
	      */
	    void
	    computeCorrespondences ();
	
	    std::string
	    getClassName () { return "FPFHEstimation"; }
	
	  protected:
	    float search_radius_;
	
	    FeaturesPtr source_features_;
	    FeaturesPtr target_features_;
	
	    KdTreePtr tree_;
	
	    NormalInPtr source_normals_;
	    NormalInPtr target_normals_;
	
	  };
	
	}
	
	template <typename PointIn> void
	pcl::FeatureCorrespondenceTest<PointIn>::computeResults ()
	{
	  if (correspondences_ == NULL)
	    return;
	
	  no_of_successes_ = 0;
	  no_of_failures_ = 0;
	
	  pcl::transformPointCloud (*source_input_,*source_transform_,ground_truths_);
	
	  for (int index = 0; index < (source_input_->points).size(); index++)
	  {
	    int corresponding_point = (*correspondences_)[index];
	    float distance_3d = pcl::euclideanDistance<PointIn, PointIn> ((target_input_->points)[corresponding_point],
	                                                                  (source_transform_->points)[index]);
	    if (distance_3d <= search_threshold_)
	    {
	      no_of_successes_++;
	    }
	    else
	    {
	      no_of_failures_++;
	    }
	  }
	}
	
	template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
	pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeNormals (float search_radius)
	{
	  NormalEstimation<PointIn, NormalT> ne_source;
	  ne_source.setInputCloud (source_input_);
	
	  KdTreePointInPtr tree_source (new KdTreeFLANN<PointIn> ());
	  ne_source.setSearchMethod (tree_source);
	
	  source_normals_ = NormalInPtr(new PointCloud<NormalT>);
	
	  ne_source.setRadiusSearch (search_radius);
	
	  ne_source.compute (*source_normals_);
	
	
	  NormalEstimation<PointIn, NormalT> ne_target;
	  ne_target.setInputCloud (target_input_);
	
	  KdTreePointInPtr tree_target (new KdTreeFLANN<PointIn> ());
	  ne_target.setSearchMethod (tree_target);
	
	  target_normals_ = NormalInPtr(new PointCloud<NormalT>);
	
	  ne_target.setRadiusSearch (search_radius);
	
	  ne_target.compute (*target_normals_);
	
	}
	
	template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
	pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::setParameters (ParameterList &params)
	{
	  if (params.find ("threshold") != params.end ())
	  {
	    float threshold = boost::lexical_cast<float>(params["threshold"]);
	    this->setSearchThreshold (threshold);
	  }
	
	  if (params.find ("searchradius") != params.end ())
	  {
	    float radius = boost::lexical_cast<float>(params["searchradius"]);
	    setRadiusSearch (radius);
	  }
	}
	
	template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
	pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
	{
	  computeNormals(0.5*search_radius_);
	
	  FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_source;
	  fpfh_source.setInputCloud (source_input_);
	  fpfh_source.setInputNormals (source_normals_);
	
	  KdTreePointInPtr tree_source (new KdTreeFLANN<PointIn> ());
	  fpfh_source.setSearchMethod (tree_source);
	
	  source_features_ = FeaturesPtr(new PointCloud<FeatureDescriptor> ());
	
	  fpfh_source.setRadiusSearch (search_radius_);
	
	  fpfh_source.compute (*source_features_);
	
	  FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_target;
	  fpfh_target.setInputCloud (target_input_);
	  fpfh_target.setInputNormals (target_normals_);
	
	  KdTreePointInPtr tree_target (new KdTreeFLANN<PointIn> ());
	  fpfh_target.setSearchMethod (tree_target);
	
	  target_features_ = FeaturesPtr(new PointCloud<FeatureDescriptor> ());
	
	  fpfh_target.setRadiusSearch (search_radius_);
	
	  fpfh_target.compute (*target_features_);
	}
	
	template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
	pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeCorrespondences ()
	{
	  if (source_features_ == NULL || target_features_ == NULL)
	    return;
	
	  tree_ = KdTreePtr(new KdTreeFLANN<FeatureDescriptor>);
	  tree_->setInputCloud (target_features_);
	
	  std::vector<int> nearest_neighbour (1,0);
	  std::vector<float> distance (1,0.0);
	
	  correspondences_ = new MapSourceTargetIndices;
	
	  //std::cerr << "source_features_ size " << (source_features_->points).size() << std::endl;
	
	  for (int index = 0; index < (source_features_->points).size(); index++)
	  {
	    int k = tree_->nearestKSearch ( (source_features_->points)[index], 1,
	                                   nearest_neighbour, distance);
	    //std::cerr << "Correspondences: " << index << " - " << nearest_neighbour[0] << std::endl;
	    (*correspondences_)[index] = nearest_neighbour[0];
	  }
	}

.. _feature_evaluation_framework:

FeatureEvaluationFramework Class
--------------------------------

The FeatureEvaluationFramework class provides a general framework for performing tests on different feature descriptor algorithms, with multiple sets of parameters, and on multiple datasets.
The primary use of this class is to run a series of many different Feature Descriptor tests, and publish the results in a suitable form.

To test individual algorithms separately, it is more convenient to directly use a derivative of FeatureCorrespondenceTest class.

Currently, the functionality of storing the test results in a meaningful way is a work in progress.

.. code-block:: cpp
   :linenos:
   
	#include <vector>
	#include <map>
	#include <fstream>
	
	#include <boost/tokenizer.hpp>
	#include <boost/algorithm/string/trim.hpp>
	
	#include <pcl/pcl_base.h>
	#include <pcl/point_cloud.h>
	#include <pcl/point_types.h>
	
	#include <pcl/kdtree/tree_types.h>
	#include <pcl/kdtree/kdtree.h>
	#include <pcl/kdtree/kdtree_flann.h>
	
	#include <pcl/features/feature.h>
	#include <pcl/features/normal_3d.h>
	#include <pcl/features/fpfh.h>
	
	#include <pcl/registration/transforms.h>
	
	#include <Eigen/Core>
	#include <Eigen/StdVector>
	
	#include "feature_test.h"
	
	namespace pcl
	{
	  /** \brief Framework class for running multiple feature correspondence trials on specified datasets and input parameters.
	    *
	    */
	  template <typename PointIn>
	  class FeatureEvaluationFramework: public PCLBase<PointIn>
	  {
	  public:
	    typedef pcl::PointCloud<PointIn> PointCloudIn;
	    typedef typename PointCloudIn::Ptr PointCloudInPtr;
	    typedef typename PointCloudIn::ConstPtr PointCloudInConstPtr;
	
	    typedef std::map <std::string, std::string> ParameterList;
	
	    typedef typename FeatureCorrespondenceTest<PointIn>::Ptr FeatureCorrespondenceTestPtr;
	
	  private:
	    /** \brief Stores a single dataset on which algorithm will be executed.
	      *
	      */
	    class CloudDataset
	    {
	    public:
	      CloudDataset () : label_("Dataset"), source_input_(), target_input_(), ground_truths_(Eigen::Matrix4f::Identity())
	      {}
	
	      CloudDataset (std::string label, PointCloudInPtr &source, PointCloudInPtr &target, Eigen::Matrix4f &ground_truths) :
	        label_(label), source_input_(source), target_input_(target), ground_truths_(ground_truths)
	      {}
	
	      std::string label_;
	
	      PointCloudInPtr source_input_;
	      PointCloudInPtr target_input_;
	
	      Eigen::Matrix4f ground_truths_;
	    };
	
	    /** \brief Stores a set of parameter values, and corresponding feature name.
	      *
	      */
	    class Trial
	    {
	    public:
	      Trial () : feature_name_("Unknown"), label_ ("Empty trial"), params_ () {}
	
	      Trial (std::string feature_name, std::string label, ParameterList &params) :
	        feature_name_(feature_name), label_(label), params_(params)
	      {}
	
	      std::string feature_name_;
	      std::string label_;
	
	      ParameterList params_;
	    };
	
	  public:
	    /** \brief Adds all feature descriptor test classes to list of tests.
	      *
	      */
	    FeatureEvaluationFramework ()
	    {
	      tests_.clear();
	      all_trials_.resize(0);
	      datasets_.resize(0);
	
	      // Build our Test registry (We'll need a line here for every feature test implemented)
	      //includeTest<PFHTest<PointIn, Normal, FPFHSignature33> > ();
	      includeTest<FPFHTest<PointIn, Normal, FPFHSignature33> > ();
	      //includeTest<MySuperAwesomeFeatureTest<PointIn, Histogram<123> > > ();
	      // and so on ..
	    }
	
	    /** \brief Stores a pair (feature name, parameterlist) in list of trials.
	      *
	      * \note The parameter list should be passed as a map<string,string> of (key,value) pairs.
	      *
	      * \note The actual parsing of the parameter value strings into respective types (int, float, etc)
	      * should be implemented in the setParameters method of the corresponding FeatureTest class.
	      */
	    void addTrial (std::string feature_name, ParameterList &params, std::string label)
	    {
	      all_trials_.push_back(Trial(feature_name, label, params));
	    }
	
	    /** \brief Reads a string of (parameter, value) pairs for a trial and adds the trial to list.
	      *
	      * \note The parameter list should be formatted as "param_name1=param_value1, param_name2=param_value2, ..."
	      */
	    void addTrial (std::string feature_name, std::string params_str, std::string label = "")
	    {
	      if (label == "") label = params_str;
	      ParameterList params;
	
	      boost::char_separator<char> sep(", ");
	      boost::tokenizer<boost::char_separator<char> > tokens(params_str, sep);
	
	      for (boost::tokenizer<boost::char_separator<char> >::iterator it = tokens.begin(); it != tokens.end(); it++)
	      {
	        size_t found = (*it).find('=');
	        if (found == std::string::npos) continue;
	        else
	        {
	          params[(*it).substr(0,found)] = (*it).substr(found+1);
	        }
	      }
	
	      addTrial(feature_name, params, label);
	    }
	
	    /** \brief Reads parameter values line-by-line from an input file
	      *
	      * \note Each line of file should have a feature name and  a set of parameters.
	      *
	      * \note Each line should be formatted as : "feature_name param_name1=param_value1, param_name2=param_value2, ... "
	      */
	    void addTrialsFromFile (std::string filename)
	    {
	      std::ifstream infile (filename.c_str());
	      std::string feature, params;
	
	      while (!infile.eof())
	      {
	        infile >> feature;
	        if (infile.eof()) break;
	        getline (infile, params);
	        boost::trim(params);
	        addTrial (feature, params);
	      }
	
	      infile.close();
	    }
	
	    /** \brief Adds a set of labelled input data to list.
	      *
	      */
	    void addDataset (std::string label, PointCloudInPtr &source, PointCloudInPtr &target, Eigen::Matrix4f &ground_truths)
	    {
	      CloudDataset data (label, source, target, ground_truths);
	      datasets_.push_back (data);
	    }
	
	    /** \brief Run each trial on each of the input dataset.
	      *
	      */
	    void runTests ()
	    {
	      //For each dataset
	      for (size_t d = 0; d < datasets_.size (); ++d)
	      {
	        // Run each trial
	        for (size_t i = 0; i < all_trials_.size (); ++i)
	        {
	          // Get the i^th set of parameters
	          const Trial & trial = all_trials_[i];
	
	          // Get a pointer to the appropriate Test class by keying on the specified feature name
	          if (tests_.find(trial.feature_name_) != tests_.end())
	          {
	            // Run the test
	            (tests_[trial.feature_name_])->setInputClouds (datasets_[d].source_input_, datasets_[d].target_input_);
	            (tests_[trial.feature_name_])->setGroundTruths (datasets_[d].ground_truths_);
	            (tests_[trial.feature_name_])->setParameters(trial.params_);
	
	            (tests_[trial.feature_name_])->computeFeatures();
	            (tests_[trial.feature_name_])->computeCorrespondences();
	            (tests_[trial.feature_name_])->computeResults();
	
	            std::cout << "----------Test Details:----------" << std::endl;
	            std::cout << "Feature Name:  " << trial.feature_name_ << std::endl;
	            std::cout << "Input Dataset: " << datasets_[d].label_ << std::endl;
	            std::cout << "Parameters:    " << all_trials_[i].label_ << std::endl;
	            std::cout << "----------Test Results:----------" << std::endl;
	            std::cout << "Input Size:    " << (datasets_[d].source_input_)->points.size () << std::endl;
	            std::cout << "Successes:     " << (tests_[trial.feature_name_])->getSuccesses () << std::endl;
	            std::cout << "Failures:      " << (tests_[trial.feature_name_])->getFailures () << std::endl;
	            std::cout << "---------------------------------" << std::endl;
	
	            std::cout << std::endl;
	          }
	          else
	          {
	            // If the "feature_name" isn't in our map, I'm assuming it returns a null ptr.  Need to double check that...
	            PCL_ERROR ("Unrecognized feature name! (%s)", trial.feature_name_.c_str());
	          }
	        }
	      }
	    }
	
	    void clearTrials ()
	    {
	      all_trials_.clear();
	    }
	
	    void clearDatasets ()
	    {
	      datasets_.clear();
	    }
	
	  private:
	
	    /** \brief Add the given test class to our registry of correspondence tests
	      *
	      */
	    template <class FeatureCorrespondenceTest>
	    void includeTest ()
	    {
	      FeatureCorrespondenceTest test;
	
	      // Every feature has a "getClassName"
	      tests_[test.getClassName ()] = typename FeatureCorrespondenceTest::Ptr  (new FeatureCorrespondenceTest);
	    }
	
	    /** \brief A list of the parameters for each trial
	      *
	      */
	    std::vector<Trial> all_trials_;
	
	    /** \brief A map from class name to the FeatureCorrespondenceTests for those tests
	      *
	      */
	    std::map<std::string, FeatureCorrespondenceTestPtr> tests_;
	
	    /** \brief Set of input data to run each test on.
	      *
	      */
	    std::vector<CloudDataset, Eigen::aligned_allocator<CloudDataset> > datasets_;
	
	  };
	
	}



.. _test_features:

Sample Program To Test FeatureEvaluationFramework
-------------------------------------------------

A toy program to test the class implementation, here source == target and ground_truths == Eigen::Matrix4f::Identity, hence expected output is all successes.

.. code-block:: cpp
   :linenos:
   
	#include <iostream>
	#include <map>
	
	#include <pcl/io/io.h>
	#include <pcl/io/pcd_io.h>
	
	#include <pcl/point_types.h>
	#include <pcl/features/fpfh.h>
	
	#include <Eigen/Core>
	
	#include "feature_evaluation_framework.h"
	
	int main()
	{
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
	  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud3(new pcl::PointCloud<pcl::PointXYZ>);
	
	  pcl::FeatureEvaluationFramework<pcl::PointXYZ> test_features;
	
	  Eigen::Matrix4f ground_truths = Eigen::Matrix4f::Identity ();
	
	  pcl::io::loadPCDFile("bun0.pcd", *cloud1);
	  test_features.addDataset("bun0.pcd", cloud1, cloud1, ground_truths);
	
	  pcl::io::loadPCDFile("bun4.pcd", *cloud2);
	  test_features.addDataset("bun4.pcd", cloud2, cloud2, ground_truths);
	
	  //pcl::io::loadPCDFile("bunny.pcd", *cloud3);
	  //test_features.addDataset("bunny.pcd", cloud3, cloud3, ground_truths);
	
	  std::string parameters = "threshold=0.01, searchradius=0.03";
	  test_features.addTrial("FPFHEstimation", parameters, parameters);
	
	  test_features.addTrialsFromFile("paramlist.txt");
	
	  test_features.runTests();
	
	  return 0;
	}

The file "paramlist.txt" contains sets of parameter values on each line:

paramlist.txt::

	FPFHEstimation threshold=0.02, searchradius=0.03
	FPFHEstimation threshold=0.04, searchradius=0.06


.. _test_output:

Test Output
-----------

On compiling and executing the above code, the output obtained is::

	$ ./test_feature 
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun0.pcd
	Parameters:    threshold=0.01, searchradius=0.03
	----------Test Results:----------
	Input Size:    397
	Successes:     397
	Failures:      0
	---------------------------------
	
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun0.pcd
	Parameters:    threshold=0.02, searchradius=0.03
	----------Test Results:----------
	Input Size:    397
	Successes:     397
	Failures:      0
	---------------------------------
	
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun0.pcd
	Parameters:    threshold=0.04, searchradius=0.06
	----------Test Results:----------
	Input Size:    397
	Successes:     397
	Failures:      0
	---------------------------------
	
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun4.pcd
	Parameters:    threshold=0.01, searchradius=0.03
	----------Test Results:----------
	Input Size:    361
	Successes:     361
	Failures:      0
	---------------------------------
	
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun4.pcd
	Parameters:    threshold=0.02, searchradius=0.03
	----------Test Results:----------
	Input Size:    361
	Successes:     361
	Failures:      0
	---------------------------------
	
	----------Test Details:----------
	Feature Name:  FPFHEstimation
	Input Dataset: bun4.pcd
	Parameters:    threshold=0.04, searchradius=0.06
	----------Test Results:----------
	Input Size:    361
	Successes:     361
	Failures:      0
	---------------------------------
	


