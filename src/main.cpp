#include "utils.hpp"

volatile bool flg_exit = false;
void SigHandle(int sig)
{
    ROS_WARN("Caught signal %d, stopping...", sig);
    flg_exit = true; // Set the flag to stop the loop
}

class FPR
{
public:
    FPR(ros::NodeHandle &nh)
    {
        // Params:
        nh.param<std::string>("input_topic", input_topic_, "/lidar_points");
        nh.param<std::string>("output_topic_fpr", output_topic_fpr, "/fpr");
        nh.param<std::string>("output_topic_voxelized", output_topic_voxelized, "/fpr_voxelized");
        nh.param<std::string>("output_topic_filtered", output_topic_filtered, "/fpr_filtered");
        nh.param<double>("voxel_size", voxel_size, .5);

        
        pub_full = nh.advertise<sensor_msgs::PointCloud2>(output_topic_fpr, 100);
        pub_voxelized = nh.advertise<sensor_msgs::PointCloud2>(output_topic_voxelized, 100);
        pub_filtered = nh.advertise<sensor_msgs::PointCloud2>(output_topic_filtered, 100);

        nh.param<bool>("from_bag", from_bag_, false);
        std::cout<<"from_bag_:"<<from_bag_<<std::endl;
        
        ROS_INFO("Publishing full to: %s", output_topic_fpr.c_str());
        ROS_INFO("Publishing voxelized to: %s", output_topic_voxelized.c_str());
        ROS_INFO("Publishing filtered to: %s", output_topic_filtered.c_str());

        for (int i = -kernel_row; i <= kernel_row; i++)
        { // row
            for (int j = -kernel_col; j <= kernel_col; j++)
            {
                if (j == 0 && i == 0)
                {
                    continue;
                }
                _kernel.push_back(std::make_pair(i, j));
            }
        }

        if(from_bag_)
        {
            sub_ = nh.subscribe(input_topic_, 10, &FPR::cloudCallback, this);
            ROS_INFO("Subscribed to: %s", input_topic_.c_str());

            //todo - read this from some param file
            N_SCAN = 32;         // number of vertical channels
            ang_res_x = 0.18f; // horizontal angular resolution in degrees
            Horizon_SCAN = 2000;  // 360° / ang_res_x° resolution
        }
        else 
        {
            nh.param<std::string>("bin_file", bin_file_, "");
            std::cout<<"Read from "<<bin_file_<<std::endl;
            //config of the hdl-64
            N_SCAN = 64;
            ang_res_x = 0.2; // horizontal angular resolution in degrees
            Horizon_SCAN = 1800; 
            processFolder(bin_file_);
        }
    }

private:

    void processFolder(std::string &dataset_folder)
    {
        std::string sequence = "03"; // sset the desired scenario (Note, there is no 01 and 08 scenarios)

        // set the fog, rain or snow
        // std::string dataset_folder = "/your_path_goes_here/data_3d_augmented/2013_05_28_drive_00" + sequence + "_sync/augmented_fog";
        // std::string dataset_folder = "/your_path_goes_here/data_3d_augmented/2013_05_28_drive_00" + sequence + "_sync/augmented_rain";
        // std::string dataset_folder = "/your_path_goes_here/data_3d_augmented/2013_05_28_drive_00" + sequence + "_sync/augmented_snow";

        std::string timestamp_path = dataset_folder + "timestamps.txt";                                                       // the timestmps for all the scans
        std::cout << "dataset_folder: " << dataset_folder << std::endl;
        std::cout << "timestamp_path: " << timestamp_path << std::endl;

        std::ifstream timestamp_file(timestamp_path, std::ifstream::in);

        std::string line;
        std::size_t line_num = 0;
        float timestamp = 0.;
        int scan = 0;
        ros::Time ros_timestamp;        
    
        // if (sequence == "02")
        // {
        //     line_num = 4391;
        // }

        signal(SIGINT, SigHandle); // Handle Ctrl+C (SIGINT)
        flg_exit = false;

        int point_dim = 6; //[x,y,z,intensity,ring,time]
        while (std::getline(timestamp_file, line))
        {
            if (!ros::ok())
            {
                break;
            }

            if (flg_exit)
                break;

            timestamp += .1; // you could take the actual time from line - std::getline(timestamp_file, line)
            ros_timestamp.fromSec(timestamp);

            std::cout << "\nscan:" << scan << std::endl;
            scan++;

            std::stringstream lidar_data_path;

            // fog version
            lidar_data_path << dataset_folder + "augmented_fog/" << std::setfill('0') << std::setw(10) << line_num << ".bin";

            // snow version
            // lidar_data_path << dataset_folder + "augmented_snow/" << std::setfill('0') << std::setw(10) << line_num << ".bin";

            // rain version
            // lidar_data_path << dataset_folder + "augmented_rain/" << std::setfill('0') << std::setw(10) << line_num << ".bin";

            std::vector<float> lidar_data;
            lidar_data = read_lidar_buffer(lidar_data_path.str()); // read the lidar data
            PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
            PointCloudXYZI::Ptr cloud_trimmed(new PointCloudXYZI);
            PointCloudXYZI::Ptr cloud_voxelized(new PointCloudXYZI);
            PointCloudXYZI::Ptr filtered(new PointCloudXYZI);

            for (std::size_t i = 0; i < lidar_data.size(); i += point_dim)
            {
                float x = lidar_data[i];
                float y = lidar_data[i + 1];
                float z = lidar_data[i + 2];
                float intensity = lidar_data[i + 3];
                float ring = lidar_data[i + 4];
                float time = lidar_data[i + 5];

                PointType p;
                p.x = x;
                p.y = y;
                p.z = z;
                p.intensity = intensity;
                p.timestamp = time;
                p.ring = ring;

                cloud->push_back(p);
            }

            line_num++;

            std::cout<<"cloud:"<<cloud->size()<<std::endl;
            // 1. range image projection
            Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> Neighbours;
            Neighbours.resize(N_SCAN, Horizon_SCAN);
            projectToRangeImage(cloud, cloud_trimmed, Neighbours);

            // 2.rank estimation
            computeRank(cloud, cloud_trimmed, Neighbours);

            // 3. rank based voxelization
            bool use_weighted = true; // false;
            Voxelize(cloud_trimmed, cloud_voxelized, voxel_size, use_weighted);
            // std::cout<<"\ncloud_voxelized:"<<cloud_voxelized->size()<<std::endl;

            // 4. example filter x% lowest rank points
            FilterByIntensityPercent(cloud_trimmed, filtered, 10.0); // drop 10%

            // publish the clouds
            sensor_msgs::PointCloud2 out_msg, out_msg2, out_msg3;
            pcl::toROSMsg(*cloud_trimmed, out_msg);    // full cloud with fpr
            pcl::toROSMsg(*cloud_voxelized, out_msg2); // voxelized
            pcl::toROSMsg(*filtered, out_msg3);        // filtered

            sensor_msgs::PointCloud2 msg;
            msg.header.frame_id = "Velodyne";  
            msg.header.stamp = ros_timestamp;

            out_msg.header = msg.header;
            out_msg2.header = msg.header;
            out_msg3.header = msg.header;

            pub_full.publish(out_msg);
            pub_voxelized.publish(out_msg2);
            pub_filtered.publish(out_msg3);

            int wait_time_ms = 50; // 0.05 second 
            cv::waitKey(wait_time_ms);
        }
        cv::destroyAllWindows();
    }

    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
        PointCloudXYZI::Ptr cloud_trimmed(new PointCloudXYZI);
        PointCloudXYZI::Ptr cloud_voxelized(new PointCloudXYZI);
        PointCloudXYZI::Ptr filtered(new PointCloudXYZI);
        pcl::fromROSMsg(*msg, *cloud);

        // ROS_INFO_STREAM("Received cloud with " << cloud->points.size() << " points.");

        // 1. range image projection
        Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> Neighbours;
        Neighbours.resize(N_SCAN, Horizon_SCAN);
        projectToRangeImage(cloud, cloud_trimmed, Neighbours);

        // 2.rank estimation
        computeRank(cloud, cloud_trimmed, Neighbours);

        // 3. rank based voxelization
        bool use_weighted = true; // false;
        Voxelize(cloud_trimmed, cloud_voxelized, voxel_size, use_weighted);
        // std::cout<<"\ncloud_voxelized:"<<cloud_voxelized->size()<<std::endl;

        // 4. example filter x% lowest rank points
        FilterByIntensityPercent(cloud_trimmed, filtered, 10.0); // drop 10%

        // publish the clouds
        sensor_msgs::PointCloud2 out_msg, out_msg2, out_msg3;
        pcl::toROSMsg(*cloud_trimmed, out_msg);    // full cloud with fpr
        pcl::toROSMsg(*cloud_voxelized, out_msg2); // voxelized
        pcl::toROSMsg(*filtered, out_msg3);        // filtered

        out_msg.header = msg->header;
        out_msg2.header = msg->header;
        out_msg3.header = msg->header;

        pub_full.publish(out_msg);
        pub_voxelized.publish(out_msg2);
        pub_filtered.publish(out_msg3);
    }

    void projectToRangeImage(const pcl::PointCloud<PointType>::Ptr &cloud,
                             pcl::PointCloud<PointType>::Ptr &cloud_trimmed, Eigen::MatrixXi &index_out)
    {
        // Init with -1 (meaning no point projected at this pixel yet)
        index_out.resize(N_SCAN, Horizon_SCAN);
        index_out.setConstant(-1);

        cv::Mat show_image, range_image = cv::Mat::zeros(N_SCAN, Horizon_SCAN, CV_8U);
        for (int idx = 0; idx < cloud->points.size(); idx++)
        {
            auto &pt = cloud->points[idx];
            float range = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z * pt.z);
            if (range < 0.1 || range > max_range)
                continue; // min-max clip

            int row = pt.ring; // from ring
            // float verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
            // row = (verticalAngle + ang_bottom) / ang_res_y;
            // if (row < 0 || row >= N_SCAN)
            //     continue;

            float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
            int col = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;

            if (col >= Horizon_SCAN)
                col -= Horizon_SCAN;
            if (col < 0 || col >= Horizon_SCAN)
                continue;

            // std::cout<<"row:"<<row<<", col:"<<col<<", range:"<<range<<std::endl;

            // Keep closest point in case of overlaps
            // if (range < range_image.at<float>(row, col))
            //     range_image.at<float>(row, col) = range;

            range_image.at<uchar>(row, col) = (range / 30) * 255; // just to show the ranges
            index_out(row, col) = idx;                            // index of the point from the original cloud
            pt.intensity = range;                                 // save the range in intensity
            cloud_trimmed->points.push_back(pt);
        }

        cv::applyColorMap(range_image, show_image, cv::COLORMAP_JET);
        // cv::flip(show_image, show_image, 0);
        cv::imshow("Range Image", show_image);
        cv::waitKey(1);

        // cv::waitKey(0);
    }

    bool projectPoint(const PointType &pt, int &row, int &col)
    {
        row = pt.ring; // from ring
        // float verticalAngle = atan2(pt.z, sqrt(pt.x * pt.x + pt.y * pt.y)) * 180 / M_PI;
        // row = (verticalAngle + ang_bottom) / ang_res_y;
        // if (row < 0 || row >= N_SCAN)
        //     return false;

        float horizonAngle = atan2(pt.x, pt.y) * 180 / M_PI;
        col = -round((horizonAngle - 90.0) / ang_res_x) + Horizon_SCAN / 2;

        if (col >= Horizon_SCAN)
            col -= Horizon_SCAN;
        if (col < 0 || col >= Horizon_SCAN)
            return false;

        return true;
    }

    void computeRank(const PointCloudXYZI::Ptr &cloud, PointCloudXYZI::Ptr &cloud_trimmed,
                     Eigen::MatrixXi &Neighbours)
    {
        cv::Mat show_image, densityMat = cv::Mat::zeros(N_SCAN, Horizon_SCAN, CV_8UC1);

        int cloud_size = cloud_trimmed->points.size();

        tbb::parallel_for(tbb::blocked_range<int>(0, cloud_size), [&](tbb::blocked_range<int> _range)
                          {
                               for (int idx = _range.begin(); idx < _range.end(); idx++)
                              //for (int idx = 0; idx < cloud_size; idx++)
                              {
                                  auto &pt = cloud_trimmed->points[idx];
                                  int row, col;
                                  if (projectPoint(pt, row, col))
                                  {
                                      float range_curr = pt.intensity;
                                      int count = 0;
                                      float rank = 0.;
                                      for (const auto &neigh : _kernel)
                                      {
                                          int y = row + neigh.first;
                                          int x = col + neigh.second;
                                          if (y < 0 || y >= N_SCAN)
                                              continue; // out of the rings

                                          if (x < 0)
                                              x = Horizon_SCAN - 1 + x;
                                          if (x >= Horizon_SCAN)
                                              x = x - Horizon_SCAN - 1;

                                          if (x < 0 || x >= Horizon_SCAN)
                                              continue;

                                          auto &idx = Neighbours(y, x);

                                          if (idx > 0) // has a projected point
                                          {
                                              const auto &p = cloud->points[idx];
                                              count++;
                                              float diff = p.intensity - range_curr;
                                              rank += std::exp(-0.5f * diff * diff);
                                          }
                                      }
                                      if (count > 0)
                                      {
                                          rank /= count;
                                      }
                                    
                                      float weight = range_curr / max_range;     // normalized 0–1 //rank *= weight;
                                      rank = (1 + rank) * (1 + weight);
                                      
                                      pt.intensity = rank;
                                      
                                      //densityMat.at<uchar>(row, col) = rank * 240.; // just for display
                                  }
                              } }); // using tbb
        // display the rank
        for (int idx = 0; idx < cloud_size; idx++)
        {
            auto &pt = cloud_trimmed->points[idx];
            int row, col;
            if (projectPoint(pt, row, col))
            {
                densityMat.at<uchar>(row, col) = pt.intensity * 100;// 64.; // just for display
            }
        }
        //cv::applyColorMap(densityMat, show_image, cv::COLORMAP_SUMMER); 
        cv::applyColorMap(densityMat, show_image, cv::COLORMAP_RAINBOW); // COLORMAP_RAINBOW
        // cv::flip(show_image, show_image, 0);
        cv::imshow("FPR", show_image);

        cv::waitKey(1);
        // cv::waitKey(0);
    }

    void Voxelize(const PointCloudXYZI::Ptr &frame_in,
                  PointCloudXYZI::Ptr &frame_out,
                  double voxel_size, bool use_weighted = false)
    {

        tsl::robin_map<Voxel, VoxelAccum, VoxelHash> grid;
        grid.reserve(frame_in->size());

        for (const auto &point : frame_in->points)
        {
            Eigen::Vector3d p(point.x, point.y, point.z);
            const auto voxel = Voxel((p / voxel_size).cast<int>());
            auto &acc = grid[voxel]; // creates new if not exists

            if (use_weighted)
            {
                // accumulate weighted sum
                acc.weighted_sum += point.intensity * p;
                acc.weight_sum += point.intensity;
            }
            else
            {
                // max intensity selection
                if (point.intensity > acc.max_intensity)
                {
                    acc.max_intensity = point.intensity;
                    acc.max_point = point;
                }
            }
        }

        frame_out->clear();
        frame_out->reserve(grid.size());

        for (const auto &[voxel, acc] : grid)
        {
            (void)voxel;
            PointType out_point;
            if (use_weighted)
            {
                Eigen::Vector3d p = acc.weighted_sum / acc.weight_sum;
                out_point.x = p.x();
                out_point.y = p.y();
                out_point.z = p.z();
                out_point.intensity = acc.max_intensity; // keep max intensity
            }
            else
            {
                out_point = acc.max_point;
            }
            frame_out->emplace_back(out_point);
        }
    }

    void FilterByIntensityPercent(const PointCloudXYZI::Ptr &frame_in,
                                  PointCloudXYZI::Ptr &frame_out,
                                  double percentage_to_drop)
    {
        /*
        percentage_to_drop - in (0 - 100)
        */
        if (frame_in->empty() || percentage_to_drop <= 0.0)
        {
            *frame_out = *frame_in; // no filtering
            return;
        }

        // Clamp percentage
        if (percentage_to_drop >= 100.0)
        {
            frame_out->clear(); // drop everything
            return;
        }

        std::vector<PointType> points(frame_in->points.begin(), frame_in->points.end());

        // How many points to drop
        size_t numToDrop = static_cast<size_t>((percentage_to_drop / 100.0) * points.size());

        // put the lowest-intensity points in the first numToDrop positions
        std::partial_sort(points.begin(),
                          points.begin() + numToDrop,
                          points.end(),
                          [](const PointType &a, const PointType &b)
                          {
                              if (fabs(a.intensity - b.intensity) < 1e-6)
                                  return a.z < b.z; // tie-breaker
                              return a.intensity < b.intensity;
                          });

        frame_out->clear();
        frame_out->reserve(points.size() - numToDrop);
        frame_out->insert(frame_out->end(),
                          points.begin() + numToDrop,
                          points.end());

        ROS_INFO_STREAM("FilterByIntensityPercent: dropped "
                        << numToDrop << " / " << points.size()
                        << " points (" << percentage_to_drop << "%)");
    }

    ros::Subscriber sub_;
    ros::Publisher pub_full, pub_voxelized, pub_filtered;
    std::string input_topic_, bin_file_;
    std::string output_topic_fpr, output_topic_voxelized, output_topic_filtered;

    float max_range = 100;
    double voxel_size = .5;
    std::vector<std::pair<int, int>> _kernel;
    bool from_bag_ = false;

    // config of the hesai
    int N_SCAN = 32;         // number of vertical channels
    float ang_res_x = 0.18f; // horizontal angular resolution in degrees
    int Horizon_SCAN = 2000;  // 360° / ang_res_x° resolution

    //config of the hdl-64
    //int N_SCAN = 64;
    //float ang_res_x = 0.2; // horizontal angular resolution in degrees
    //int Horizon_SCAN = 1800; 
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fpr");
    ros::NodeHandle nh("~");

    FPR obj(nh);

    ros::spin();
    return 0;
}