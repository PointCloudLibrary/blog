Feature Evaluation Framework Source Code
========================================

.. _evaluation_framework_6_24:

FeatureEvaluationFramework Class
--------------------------------

The Framework class, for testing Feature Descriptor algorithms. Latest additions: enabling preprocessing of input clouds, running tests for varying leaf sizes (of VoxelGrid filter) and storing output in a TestResult object.

.. code-block:: cpp
   :linenos:
   
   #include <vector>
   #include <map>
   #include <fstream>
   #include <string>
   
   #include <boost/tokenizer.hpp>
   #include <boost/algorithm/string/trim.hpp>
   #include <boost/timer.hpp>
   
   #include <pcl/pcl_base.h>
   #include <pcl/point_cloud.h>
   #include <pcl/point_types.h>
   
   #include <pcl/kdtree/tree_types.h>
   #include <pcl/kdtree/kdtree.h>
   #include <pcl/kdtree/kdtree_flann.h>
   
   #include <pcl/filters/voxel_grid.h>
   
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
   
       class LeafSize
       {
       public:
         LeafSize () : x_(0.05), y_(0.05), z_(0.05) {}
   
         LeafSize (float x, float y, float z) : x_(x), y_(y), z_(z) {}
   
         float x_, y_, z_;
       };
   
       class TestResult
       {
       public:
         TestResult () {}
   
         void print ()
         {
           std::cout << "----------Test Details:----------" << std::endl;
           std::cout << "Feature Name:  " << feature_name_ << std::endl;
           std::cout << "Input Dataset: " << dataset_label_ << std::endl;
           std::cout << "Parameters:    " << trial_label_ << std::endl;
           if (done_preprocessing_)
             std::cout << "Leaf size:     "
             << leaf_size_x_ << " "
             << leaf_size_y_ << " "
             << leaf_size_z_ << " "
             << std::endl;
           std::cout << "----------Test Results:----------" << std::endl;
           std::cout << "Source Size:   " << preprocessed_source_size_ << std::endl;
           std::cout << "Target Size:   " << preprocessed_target_size_ << std::endl;
           std::cout << "Successes:     " << successes_ << std::endl;
           std::cout << "Failures:      " << failures_ << std::endl;
           std::cout << "Time Taken For Feature Computations:" << std::endl;
           std::cout << "  Source:      " << time_source_ << std::endl;
           std::cout << "  Target:      " << time_target_ << std::endl;
           std::cout << "  Total:       " << time_features_ << std::endl;
           std::cout << "---------------------------------" << std::endl;
   
           std::cout << std::endl;
         }
   
         std::string feature_name_, dataset_label_, trial_label_;
         float leaf_size_x_, leaf_size_y_, leaf_size_z_;
         int source_size_, target_size_;
         bool done_preprocessing_;
         int preprocessed_source_size_, preprocessed_target_size_;
         int successes_, failures_;
         double time_features_, time_source_, time_target_;
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
         leaf_sizes_.resize(0);
   
         log_file_ = "test_results.txt";
   
         do_preprocessing_ = false;
         verbose_ = true;
   
         // Build our Test registry (We'll need a line here for every feature test we've implemented)
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
   
       /** \brief Controls the preprocessing (downsampling) of input clouds before running the tests
         *
         */
       void setPreProcessing (bool flag)
       {
         do_preprocessing_ = flag;
       }
   
       void addLeafSize (float x, float y, float z)
       {
         leaf_sizes_.push_back( LeafSize(x,y,z) );
       }
   
       void doPreProcessing (size_t dataset_index, size_t leaf_index, PointCloudInPtr& preprocessed_source, PointCloudInPtr& preprocessed_target)
       {
         pcl::VoxelGrid<PointIn> vox_grid;
         vox_grid.setLeafSize (leaf_sizes_[leaf_index].x_, leaf_sizes_[leaf_index].y_, leaf_sizes_[leaf_index].z_);
   
         vox_grid.setInputCloud (datasets_[dataset_index].source_input_);
         vox_grid.filter (*preprocessed_source);
   
         vox_grid.setInputCloud (datasets_[dataset_index].target_input_);
         vox_grid.filter (*preprocessed_target);
       }
   
       TestResult runSingleTest (size_t dataset_index, size_t trial_index, size_t leaf_index)
       {
         const Trial & trial = all_trials_[trial_index];
   
         if (tests_.find(trial.feature_name_) == tests_.end())
         {
           PCL_ERROR ("Unrecognized feature name! (%s)", trial.feature_name_.c_str());
           return TestResult();
         }
   
         PointCloudInPtr preprocessed_source, preprocessed_target;
         if (do_preprocessing_)
         {
           preprocessed_source = PointCloudInPtr(new PointCloud<PointIn>);
           preprocessed_target = PointCloudInPtr(new PointCloud<PointIn>);
   
           if (verbose_) std::cout << "Preprocessing input clouds" << std::endl;
           doPreProcessing (dataset_index, leaf_index, preprocessed_source, preprocessed_target);
         }
         else
         {
           preprocessed_source = datasets_[dataset_index].source_input_;
           preprocessed_target = datasets_[dataset_index].target_input_;
         }
   
         if (verbose_) std::cout << "Set input clouds" << std::endl;
         (tests_[trial.feature_name_])->setInputClouds (preprocessed_source, preprocessed_target);
   
         if (verbose_) std::cout << "Set ground truths" << std::endl;
         (tests_[trial.feature_name_])->setGroundTruths (datasets_[dataset_index].ground_truths_);
   
         if (verbose_) std::cout << "Set parameters" << std::endl;
         (tests_[trial.feature_name_])->setParameters(trial.params_);
   
         double time_normals, time_source, time_target;
         if (verbose_) std::cout << "Computing features" << std::endl;
         boost::timer time_1;
         (tests_[trial.feature_name_])->computeFeatures(time_source, time_target);
         double time_features = time_1.elapsed();
         if (verbose_) std::cout << "Time taken: " << time_features << std::endl;
   
         if (verbose_) std::cout << "Computing correspondences" << std::endl;
         (tests_[trial.feature_name_])->computeCorrespondences();
   
         if (verbose_) std::cout << "Computing results" << std::endl;
         (tests_[trial.feature_name_])->computeResults();
   
         TestResult result;
   
         result.feature_name_ = trial.feature_name_;
         result.dataset_label_ = datasets_[dataset_index].label_;
         result.trial_label_ = all_trials_[trial_index].label_;
         result.leaf_size_x_ = leaf_sizes_[leaf_index].x_;
         result.leaf_size_y_ = leaf_sizes_[leaf_index].y_;
         result.leaf_size_z_ = leaf_sizes_[leaf_index].z_;
         result.source_size_ = (datasets_[dataset_index].source_input_)->points.size ();
         result.target_size_ = (datasets_[dataset_index].target_input_)->points.size ();
         result.done_preprocessing_ = do_preprocessing_;
         result.preprocessed_source_size_ = (preprocessed_source)->points.size ();
         result.preprocessed_target_size_ = (preprocessed_target)->points.size ();
         result.successes_ = (tests_[trial.feature_name_])->getSuccesses ();
         result.failures_ = (tests_[trial.feature_name_])->getFailures ();
         result.time_features_ = time_features;
         result.time_source_ = time_source;
         result.time_target_ = time_target;
   
         if (verbose_)
         {
           result.print ();
         }
   
         return result;
       }
   
       /** \brief Run each trial on each of the input dataset.
         *
         */
       void runAllTests ()
       {
         //For each dataset
         for (size_t d = 0; d < datasets_.size (); ++d)
         {
           // Run each trial
           for (size_t i = 0; i < all_trials_.size (); ++i)
           {
             if (do_preprocessing_)
             {
               // For each set of leaf size
               for (size_t l = 0; l < leaf_sizes_.size(); ++l)
               {
                 TestResult result = runSingleTest (d, i, l);
                 //do something with result here
               }
             }
             else
             {
               TestResult result = runSingleTest (d, i, 0);
             }
           }
         }
       }
   
       /** \brief Run tests on all sets of leaf sizes for downsampling, on given dataset and trial.
         *
         * \note Define similar functions for running tests on all trials, or all datasets.
         */
       void runTestsForLeafs (size_t dataset_index, size_t trial_index)
       {
         std::ofstream logfile (log_file_.c_str());
   
         logfile << "Feature Name: " << all_trials_[trial_index].feature_name_ << std::endl;
         logfile << "Parameters:   " << all_trials_[trial_index].label_ << std::endl;
         logfile << "Dataset:      " << datasets_[dataset_index].label_ << std::endl;
         logfile << "Source size:  " << (datasets_[dataset_index].source_input_)->points.size () << std::endl;
         logfile << "Testcases: Leaf size, Preprocessed Source Size, Preprocessed Target Size, Time for Source, Time for Target, Time for Features" << std::endl;
         logfile << std::endl;
   
         if (!do_preprocessing_)
         {
           PCL_ERROR ("Set Preprocessing Flag First!");
           return;
         }
   
         for (size_t l = 0; l < leaf_sizes_.size(); l++)
         {
           TestResult result = runSingleTest (dataset_index, trial_index, l);
           logfile << result.leaf_size_x_ << ", " << result.preprocessed_source_size_ << ", " << result.preprocessed_target_size_ << ", ";
           logfile << result.time_source_ << ", " << result.time_target_ << ", " << result.time_features_ << std::endl;
         }
   
         logfile << "EndFile" << std::endl;
         logfile.close();
       }
   
       void setLogFile (std::string s)
       {
         log_file_ = s;
       }
   
       void setVerbose (bool flag)
       {
         verbose_ = flag;
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
         // I think we can make FeatureCorrespondenceTest a friend and expose this method
         tests_[test.getClassName ()] = typename FeatureCorrespondenceTest::Ptr  (new FeatureCorrespondenceTest);
       }
   
       /** \brief A list of the parameters for each trial
         */
       std::vector<Trial> all_trials_;
   
       /** \brief A map from class name to the FeatureCorrespondenceTests for those tests
         */
       std::map<std::string, FeatureCorrespondenceTestPtr> tests_;
   
       /** \brief Set of input data to run each test on.
         */
       std::vector<CloudDataset, Eigen::aligned_allocator<CloudDataset> > datasets_;
   
       /** \brief Flag for controlling preprocessing of input clouds
         */
       bool do_preprocessing_;
   
       /** \brief Leaf size for downsampling point cloud using VoxelGrid
         */
       std::vector<LeafSize> leaf_sizes_;
   
       /** \brief File for recording test outputs
         */
       std::string log_file_;
   
       /** \brief Control console output during execution of tests
         */
       bool verbose_;
     };
   
   }
      
.. _feature_test_6_24:

FeatureCorrespondenceTest and FPFHTest class
--------------------------------------------

.. code-block:: cpp
   :linenos:
   
   #include <vector>
   #include <map>
   
   #include <boost/shared_ptr.hpp>
   #include <boost/lexical_cast.hpp>
   #include <boost/timer.hpp>
   
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
       getClassName () { return "FeatureTest"; }
   
       virtual void
       setParameters (ParameterList params) {}
   
       virtual void
       computeFeatures (double&, double&) {}
   
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
   
       /** \brief Compute the FPFH feature descriptors of source and target clouds, and return the time taken for both source and target features
         *
         */
       void
       computeFeatures (double& time_source, double& time_target);
   
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
   pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures (double& time_source, double& time_target)
   {
     std::cout << "FPFHTest: computing normals" << std::endl;
     computeNormals(0.5*search_radius_);
   
     FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_source;
     fpfh_source.setInputCloud (source_input_);
     fpfh_source.setInputNormals (source_normals_);
   
     KdTreePointInPtr tree_source (new KdTreeFLANN<PointIn> ());
     fpfh_source.setSearchMethod (tree_source);
   
     source_features_ = FeaturesPtr(new PointCloud<FeatureDescriptor> ());
   
     fpfh_source.setRadiusSearch (search_radius_);
   
     std::cout << "FPFHTest: computing source features" << std::endl;
     boost::timer time_1;
     fpfh_source.compute (*source_features_);
     time_source = time_1.elapsed();
   
     FPFHEstimation<PointIn, NormalT, FeatureDescriptor> fpfh_target;
     fpfh_target.setInputCloud (target_input_);
     fpfh_target.setInputNormals (target_normals_);
   
     KdTreePointInPtr tree_target (new KdTreeFLANN<PointIn> ());
     fpfh_target.setSearchMethod (tree_target);
   
     target_features_ = FeaturesPtr(new PointCloud<FeatureDescriptor> ());
   
     fpfh_target.setRadiusSearch (search_radius_);
   
     std::cout << "FPFHTest: computing target features" << std::endl;
     boost::timer time_2;
     fpfh_target.compute (*target_features_);
     time_target = time_2.elapsed();
   }
   
   template <typename PointIn, typename NormalT, typename FeatureDescriptor> void
   pcl::FPFHTest<PointIn, NormalT, FeatureDescriptor>::computeFeatures ()
   {
     double t1, t2;
     computeFeatures (t1, t2);
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

.. _test_feature_6_24:

Main Function To Test Code
--------------------------

.. code-block:: cpp
   :linenos:
   
   #include <iostream>
   #include <map>
   #include <fstream>
   
   #include <pcl/io/io.h>
   #include <pcl/io/pcd_io.h>
   
   #include <pcl/point_types.h>
   #include <pcl/features/fpfh.h>
   #include <pcl/filters/passthrough.h>
   
   #include <Eigen/Core>
   
   #include "feature_evaluation_framework.h"
   
   int main()
   {
     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZRGB>);
   
     pcl::FeatureEvaluationFramework<pcl::PointXYZRGB> test_features;
   
     Eigen::Matrix4f ground_truths = Eigen::Matrix4f::Identity ();
   
     pcl::io::loadPCDFile("../conference_room/cloud_000.pcd", *cloud1);
   
     test_features.addDataset("cloud_000.pcd", cloud1, cloud1, ground_truths);
   
     std::cout << "Loaded input file to cloud" << std::endl;
   
     std::string parameters = "threshold=0.01, searchradius=0.003";
     test_features.addTrial ("FPFHEstimation", parameters, parameters);
   
     test_features.setVerbose (true);
     test_features.setPreProcessing (true);
     test_features.setLogFile ("variation-with-leaf-sizes.txt");
   
     std::ifstream leaf_file ("leafsizes.txt");
   
     float leaf_x, leaf_y, leaf_z;
   
     while (!leaf_file.eof())
     {
       leaf_file >> leaf_x >> leaf_y >> leaf_z;
       test_features.addLeafSize (leaf_x, leaf_y, leaf_z);
     }
     //test_features.addTrialsFromFile("paramlist.txt");
   
     test_features.runTestsForLeafs(0,0);
   
     return 0;
   }

      
