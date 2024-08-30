
// System
#include <iostream>
#include <vector>
#include <filesystem>
#include <string>
#include <random>
#include <algorithm>
#include <iterator>
#include <limits>
#include <cmath>
#include <numeric>


// Local
#include"PointCloud.hpp"
#include"../include/nanoflann.hpp"



namespace fs = std::filesystem;
using namespace nanoflann;
typedef KDTreeSingleIndexAdaptor < L2_Simple_Adaptor<double, PointCloud2<double>>,
    PointCloud2<double>, 3 > kdTree;


/// <summary>
/// Get all the files in the provided path.
/// </summary>
/// <param name="inPath"></param> The provided path.
/// <param name="files"></param>  The names of the files in the directory.
/// <returns></returns>
size_t getAllFiles(std::string inPath, std::vector<std::string>& files)
{
    try
    {
        if (fs::exists(inPath) && fs::is_directory(inPath))
        {
            for (const auto& entry : fs::recursive_directory_iterator(inPath))
            {
                if (fs::is_regular_file(entry))
                {
                    //std::cout << entry.path() << std::endl;
                    files.push_back(entry.path().string());
                }
            }
        }
        else
        {
            std::cerr << "Directory does not exist or is not a directory" << std::endl;
        }
    }
    catch (const fs::filesystem_error& e)
    {
        std::cerr << e.what() << std::endl;
    }
    return files.size();
}


/// <summary>
/// Create a folder/directory
/// </summary>
/// <param name="dirPath"></param> The provided name of the directory.
/// <returns></returns>
bool createDirectory(const std::string& dirPath)
{
    if (!fs::create_directory(dirPath))
    {
        std::cerr << "Directory creation failure." << std::endl;
        return false;
    }
    return true;
}

void select_100_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0;
    for (auto it = in.begin(); it != in.end(); ++it)
    {
        //if (counter % 5 != 4)
        {
            out.push_back(*it);
        }
        counter++;
    }
}

void select_80_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0;
    for (auto it = in.begin(); it != in.end(); ++it) 
    {
        if (counter % 5 != 4)
        {
            out.push_back(*it); 
        }
        counter++; 
    }
}

void select_60_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0; 
    for (auto it = in.begin(); it != in.end(); ++it) 
    {
        if (counter % 5 < 3) 
        { 
            out.push_back(*it); 
        }
        counter++; 
    }
}

void select_40_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0;
    for (auto it = in.begin(); it != in.end(); ++it)
    {
        if (counter % 5 == 1 || counter % 5 == 3)
        {
            out.push_back(*it);
       }
        counter++;
    }
}

void select_20_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0;

    for (auto it = in.begin(); it != in.end(); ++it)
    {
        if (counter % 5 == 0)
        {
            out.push_back(*it); 
        }
        counter++; 
    }
}

void select_10_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0; 

    for (auto it = in.begin(); it != in.end(); ++it)
    {
        if (counter % 10 == 0)
        {
            out.push_back(*it); 
        }
        counter++; 
    }
}

void select_05_percent(std::set<double> in, std::vector<double>& out)
{
    int counter = 0;
    for (auto it = in.begin(); it != in.end(); ++it)
    {
        if (counter % 20 == 0)
        {
            out.push_back(*it); 
        }
        counter++;
    }
}

std::vector<int> get_pts_indices(std::vector<double> gps_time, std::map<double, int> pts_map)
{
    assert(gps_time.size() <= pts_map.size());

    std::vector<int> pt_indices;

    for (const auto& gps : gps_time)
    {
        auto it = pts_map.find(gps);
        if (it != pts_map.end())
        {
            pt_indices.push_back(it->second); 
        }
    }

    return pt_indices;
}


void seperate_ground_pts(std::vector<int> pt_indices, PointCloud* cloud, std::vector<dPoint3D>& veg_pts,
    std::vector<dPoint3D>& gd_pts)
{
    if (pt_indices.empty())
    {
        return;
    }

    for (const auto& pt : pt_indices)
    {
        dPoint3D curr_pt;
        curr_pt = cloud->m_Points[pt];
        if (cloud->m_PtsClassInfo[pt] == 2)
        {
            gd_pts.push_back(curr_pt);
        }
        if (cloud->m_PtsClassInfo[pt] == 1)
        {
            veg_pts.push_back(curr_pt); 
        }
    }
    return;
}



void output_points(std::string output_path, PointCloud* cloud, std::vector<int> pt_indices)
{
    if (pt_indices.empty())
    {
        return;
    }

    FILE* out_file = fopen(output_path.c_str(), "w");

    for (const auto& id : pt_indices)
    {
        dPoint3D curr_pt;
        curr_pt = cloud->m_Points[id];
        fprintf(out_file, "%lf %lf %lf %lf %d\n",
            curr_pt.x, curr_pt.y, curr_pt.z,
            //cloud->m_PtsGnssTime[id] - 330440791.21733969,
            cloud->m_PtsGnssTime[id] - 330000000,
            cloud->m_PtsClassInfo[id]);
    }
    fclose(out_file);

    return;
}



/// <summary>
/// Finds the median in the provided elements in the provided container.
/// </summary>
/// <param name="vec"></param> The container of the provided elements.
/// <returns></returns> Median value.
double findMedian(std::vector<double> vec)
{
    if (vec.empty())
    {
        return 0; // Return 0 or handle as error for empty vector
    }

    size_t size = vec.size();
    std::sort(vec.begin(), vec.end()); // Sort the vector

    float median;
    if (size % 2 == 0)
    {
        // If the size is even, median is the average of the two middle elements
        median = (vec[size / 2 - 1] + vec[size / 2]) / 2;
    }
    else
    {
        // If the size is odd, median is the middle element
        median = vec[size / 2];
    }

    return median;
}


/// <summary>
/// Calculate the Std.Dev. of the provided elements in the container.
/// </summary>
/// <param name="vec"></param> The container in which holds the elements.
/// <returns></returns> The calculated Std.Dev. of the elements.
double calculateStandardDeviation(std::vector<double>& vec)
{
    if (vec.empty())
    {
        return 0; // Return 0 or handle as error for empty vector
    }

    float mean = std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
    float sq_sum = std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0f,
        std::plus<>(), [mean](float a, float b) {
            return (a - mean) * (b - mean);
        });

    return std::sqrt(sq_sum / vec.size());
}


/// <summary>
/// Calculate the Shannon index of the provided elements in the container.
/// </summary>
/// <param name="vec"></param> The container in which holds the elements.
/// <returns></returns> The calculated Shannon index.
double calculateShannonIndex(const std::vector<double>& vec)
{
    if (vec.empty())
    {
        return 0; // Handle empty vector case
    }

    float total = std::accumulate(vec.begin(), vec.end(), 0.0f);
    float shannonIndex = 0.0f;

    for (float value : vec)
    {
        if (value > 0)
        { // Ensure value is positive to avoid log of zero
            float proportion = value / total;
            shannonIndex -= proportion * std::log(proportion);
        }
    }

    return shannonIndex;
}


/// <summary>
/// Calculate the Skewness of the provided elements in the container.
/// </summary>
/// <param name="vec"></param> The container in which holds the elements.
/// <returns></returns> The calculated Skewness.
double calculateSkewness(const std::vector<double>& vec)
{
    if (vec.size() < 3)
    {
        return 0; // Skewness is not defined for n < 3
    }

    float mean = std::accumulate(vec.begin(), vec.end(), 0.0f) / vec.size();
    float stdev = std::sqrt(std::inner_product(vec.begin(), vec.end(), vec.begin(), 0.0f,
        std::plus<>(), [mean](float a, float b) {
            return (a - mean) * (b - mean);
        }) / vec.size());

    double skewness = 0.0f;
    for (float value : vec)
    {
        skewness += std::pow((value - mean) / stdev, 3);
    }

    skewness *= static_cast<double>(vec.size()) / ((vec.size() - 1) * (vec.size() - 2));

    return skewness;
} 


/// <summary>
/// Calculate the Kurtosis of the provided elements in the container.
/// </summary>
/// <param name="data"></param> The container in which holds the elements.
/// <returns></returns> The calculated Kurtosis.
double calculateKurtosis(const std::vector<double>& data)
{
    if (data.size() < 4)
    {
        //throw std::invalid_argument("Data set too small for kurtosis calculation");
        return 0.0;
    }

    double mean = std::accumulate(data.begin(), data.end(), 0.0) / data.size();

    double fourthMoment = 0.0;
    double variance = 0.0;

    for (double value : data)
    {
        double diff = value - mean;
        variance += diff * diff;
        fourthMoment += diff * diff * diff * diff;
    }

    variance /= data.size();
    fourthMoment /= data.size();

    double stdDev = std::sqrt(variance);

    double n = data.size();
    double kurtosis = ((n * (n + 1)) / ((n - 1) * (n - 2) * (n - 3))) *
        (fourthMoment / (stdDev * stdDev * stdDev * stdDev)) -
        (3 * (n - 1) * (n - 1)) / ((n - 2) * (n - 3));

    return kurtosis;
}

/// <summary>
/// Calculate the 25th percentile value of the provided elements.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <returns></returns> The calculated value.
double calculate_Hp25(const std::vector<double> heights)
{
    double hp25 = 0.0;
    if (heights.empty()) return hp25;

    std::vector<double> elevation = heights;

    std::sort(elevation.begin(), elevation.end());

    double index = 0.25 * (elevation.size() - 1);

    // Handling non-integer index by interpolating between values
    int lowerIndex = static_cast<int>(index);
    int upperIndex = lowerIndex + 1;
    float weight = index - lowerIndex;

    // Step 3: Retrieve or interpolate the value
    if (upperIndex < elevation.size())
    {
        // Interpolate
        hp25 = elevation[lowerIndex] * (1 - weight) + elevation[upperIndex] * weight;
    }
    else
    {
        // Directly retrieve the value
        return elevation[lowerIndex];
    }

    return hp25;
}


/// <summary>
/// Calculate the 50th percentile value of the provided elements.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <returns></returns> The calculated value.
double calculate_Hp50(const std::vector<double> heights)
{
    double hp50 = 0.0;

    if (heights.empty()) return hp50;

    std::vector<double> elevation = heights;

    // Step 1: Sort the vector
    std::sort(elevation.begin(), elevation.end());

    size_t n = elevation.size();

    // Step 2: Calculate the median
    if (n % 2 == 0)
    {
        // Even number of elements
        // Median is the average of the two middle numbers
        hp50 = (elevation[n / 2 - 1] + elevation[n / 2]) / 2.0;
    }
    else
    {
        // Odd number of elements
        // Median is the middle number
        hp50 = elevation[n / 2];
    }

    return hp50;
}

/// <summary>
/// Calculate the 75th percentile value of the provided elements.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <returns></returns> The calculated value.
double calculate_Hp75(const std::vector<double> heights)
{
    double hp75 = 0.0;
    if (heights.empty()) return hp75;

    std::vector<double> elevation = heights;

    std::sort(elevation.begin(), elevation.end());

    double index = 0.75 * (elevation.size() - 1);

    // Handling non-integer index by interpolating between values
    int lowerIndex = static_cast<int>(index);
    int upperIndex = lowerIndex + 1;
    float weight = index - lowerIndex;

    // Step 3: Retrieve or interpolate the value
    if (upperIndex < elevation.size())
    {
        // Interpolate
        hp75 = elevation[lowerIndex] * (1 - weight) + elevation[upperIndex] * weight;
    }
    else
    {
        // Directly retrieve the value
        return elevation[lowerIndex];
    }

    return hp75;
}

/// <summary>
/// Calculate the 95th percentile value of the provided elements.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <returns></returns> The calculated value.
double calculate_Hp95(std::vector<double> heights)
{
    double hp95 = 0.0;

    if (heights.empty()) return hp95;

    std::vector<double> elevation = heights;

    std::sort(elevation.begin(), elevation.end());

    double index = 0.95 * (elevation.size() - 1);

    // Handling non-integer index by interpolating between values
    int lowerIndex = static_cast<int>(index);
    int upperIndex = lowerIndex + 1;
    float weight = index - lowerIndex;

    // Step 3: Retrieve or interpolate the value
    if (upperIndex < elevation.size())
    {
        // Interpolate
        hp95 = elevation[lowerIndex] * (1 - weight) + elevation[upperIndex] * weight;
    }
    else
    {
        // Directly retrieve the value
        return elevation[lowerIndex];
    }

    return hp95;
}

/// <summary>
/// Calculate the density above mean value.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <param name="mean_height"></param> The mean height of the data.
/// <returns></returns> The calculdated value.
int calculate_density_above_mean_z(const std::vector<double>& heights, double mean_height)
{
    int count = 0;
    for (auto& height : heights)
    {
        if (height > mean_height)
        {
            count++;
        }
    }
    return count;
}

/// <summary>
/// Calculate the band ratio of the provided value.
/// </summary>
/// <param name="heights"></param> The container in which holds the elements.
/// <param name="brs"></param> The obtained band ratios.
void calculate_br(std::vector<double>& heights, std::vector<double>& brs)
{
    double br_1 = 0.0;
    double br_1_2 = 0.0;
    double br_2_3 = 0.0;
    double br_3 = 0.0;
    double br_3_4 = 0.0;
    double br_4_5 = 0.0;
    double br_5 = 0.0;
    double br_5_20 = 0.0;
    double br_20 = 0.0;

    for (auto& h : heights)
    {
        if (h < 1.0)
        {
            br_1 += 1.0;
        }
        if (h > 1.0 && h < 2.0)
        {
            br_1_2 += 1.0;
        }
        if (h > 2.0 && h < 3.0)
        {
            br_2_3 += 1.0;
        }
        if (h > 3.0)
        {
            br_3 += 1.0;
        }
        if (h > 3.0 && h < 4.0)
        {
            br_3_4 += 1.0;
        }
        if (h > 4.0 && h < 5.0)
        {
            br_4_5 += 1.0;
        }
        if (h < 5.0)
        {
            br_5 += 1.0;
        }
        if (h > 5.0 && h < 20.0)
        {
            br_5_20 += 1.0;
        }
        if (h > 20.0)
        {
            br_20 += 1.0;
        }
    }

    br_1 /= heights.size();
    br_1_2 /= heights.size();
    br_2_3 /= heights.size();
    br_3 /= heights.size();
    br_3_4 /= heights.size();
    br_4_5 /= heights.size();
    br_5 /= heights.size();
    br_5_20 /= heights.size();
    br_20 /= heights.size();


    brs.push_back(br_1);
    brs.push_back(br_1_2);
    brs.push_back(br_2_3);
    brs.push_back(br_3);
    brs.push_back(br_3_4);
    brs.push_back(br_4_5);
    brs.push_back(br_5);
    brs.push_back(br_5_20);
    brs.push_back(br_20);
}


/// <summary>
/// Calculate the sigma of height.
/// </summary> 
/// <param name="heights"></param> The container in which holds the elements.
/// <param name="mean_height"></param> The mean height.
/// <returns></returns> The calculated value.
double calculate_sigma_z(const std::vector<double>& heights, double mean_height)
{
    std::vector<double> residuals;
    for (auto& h : heights)
    {
        double r = h - mean_height;
        residuals.push_back(r);
    }

    return calculateStandardDeviation(residuals);
}

int main(int argc, char** argv)
{
    std::string in_las_directory = "D:\\Projects\\Repository\\Jinhu-Wang\\UpScaling\\Woodland_1m\\Input\\";
    std::string out_directory = "D:\\Projects\\Repository\\Jinhu-Wang\\UpScaling\\Woodland_1m\\Output\\";
    
    std::string out_downsampled = out_directory + "Downsampled\\";
 
    //std::string down_sample_scale = "100";
    //std::string down_sample_scale = "80";
    //std::string down_sample_scale = "60";
    //std::string down_sample_scale = "40";
    //std::string down_sample_scale = "20"; 
    //std::string down_sample_scale = "10";
    std::string down_sample_scale = "5";
    std::string scale_name = out_downsampled + down_sample_scale + "\\";



    std::vector<std::string>files; 
    getAllFiles(in_las_directory, files); 
    std::cout << "There are: " << files.size() << " files in the folder." << std::endl;
    
    std::string outMetricsFile = out_directory + "new_metrics_"+down_sample_scale +".txt";
    FILE* outFileMetrics = fopen(outMetricsFile.c_str(), "w");

    for (auto& file : files)
    {
        std::vector<dPoint3D> Points;

        LASreadOpener readOpener; 
        readOpener.set_file_name(file.c_str());
        LASreader* reader = readOpener.open();
        
        std::vector<dPoint3D> NormalizedVegetationPts;

        CLazPointData* cloud = new CLazPointData();
        if (!cloud || !cloud->LoadFile(file.c_str()))
        {
            std::cout << "Error in las/laz file reading." << std::endl;
            return 1;
        }

        std::vector<double> gps_05_percent; 

        select_05_percent(cloud->m_gps_time_sorted, gps_05_percent);   
        
        std::vector<dPoint3D> veg_pts;
        std::vector<dPoint3D> gd_pts;
        
        std::vector<int> pt_indices = get_pts_indices(gps_05_percent, cloud->m_map_gpstime_pt); 
       
        std::filesystem::path path_obj(file); 
        std::string curr_file_name = path_obj.filename().string();
        std::string output_pts_path = scale_name  + curr_file_name + ".xyz";
        output_points(output_pts_path.c_str(), cloud, pt_indices);

        seperate_ground_pts(pt_indices, cloud, veg_pts, gd_pts);
        
        if (veg_pts.empty() || gd_pts.empty())
        {
            continue;
        }

        PointCloud2<double> kdPts;
        kdPts.pts.resize(gd_pts.size()); 
        int index = 0;
        for (auto& p : gd_pts)
        {
            kdPts.pts[index].x = p.x;
            kdPts.pts[index].y = p.y;
            kdPts.pts[index].z = 0.0;
            index++;
        }
        kdTree* tree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();

        const size_t ret_num = 1;
        std::vector<size_t> ret_index(ret_num);
        std::vector<double> out_dist_sqr(ret_num);

        for (auto& p : veg_pts)
        {
            double queryPt[3];
            queryPt[0] = p.x; 
            queryPt[1] = p.y;
            queryPt[2] = 0.0;

            tree->knnSearch(&queryPt[0], ret_num, &ret_index[0], &out_dist_sqr[0]);
            
            size_t closest_pt_id = ret_index[0];
            dPoint3D currPt = p;
            currPt.z = p.z - gd_pts[closest_pt_id].z; 

            NormalizedVegetationPts.push_back(currPt); 
        }
        
        double maxHeight = (std::numeric_limits<double>::lowest)(); 
        double minHeight = (std::numeric_limits<double>::max)(); 

        double heightSum = 0.0; 
        std::vector<double> heights; 
        for (auto& pt : NormalizedVegetationPts) 
        { 
            if (pt.z > maxHeight) 
            { 
                maxHeight = pt.z; 
            }
            if (pt.z < minHeight) 
            {
                minHeight = pt.z; 
            }

            heightSum += pt.z; 
            heights.push_back(pt.z); 
        }
        heightSum /= NormalizedVegetationPts.size(); 

        double deltaHeight = maxHeight - minHeight; 
        

        // Metrics;
        float penetration_ratio = gd_pts.size() * 1.0 / (gd_pts.size() * 1.0 + veg_pts.size() * 1.0); 
        float height_max = maxHeight; 
        float height_mean = heightSum; 
        float height_median = findMedian(heights); 
        float height_stddev = calculateStandardDeviation(heights);  
        float height_shannon_index = calculateShannonIndex(heights);  
        float height_skewness = calculateSkewness(heights);  
        float height_kurtosis = calculateKurtosis(heights);  
        //float height_25 = deltaHeight * 0.25;
        float height_25 = calculate_Hp25(heights); 
        //float height_50 = deltaHeight * 0.5;
        float height_50 = calculate_Hp50(heights); 
        //float height_75 = deltaHeight * 0.75;
        float height_75 = calculate_Hp75(heights); 
        //float height_95 = deltaHeight * 0.95;
        float height_95 = calculate_Hp95(heights); 

        int density_above_mean_z = calculate_density_above_mean_z(heights, heightSum);  

        std::vector<double> brs; 
        calculate_br(heights, brs); 

        float coeff_var_z = height_stddev / height_mean; 

        float height_variance = std::pow(height_stddev, 2);
        float sigma_z = calculate_sigma_z(heights, height_mean);  

        //float pt_density = (veg_pts.size() + gd_pts.size())/(100.0);
        size_t pt_veg_num = veg_pts.size();

        fprintf(outFileMetrics, "%lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %d\n",
            height_max, height_mean, height_median, height_25, height_50, height_75, height_95, 
            penetration_ratio, density_above_mean_z, 
            brs[0], brs[1], brs[2], brs[3], brs[4], brs[5], brs[6], brs[7], brs[8], 
            coeff_var_z, height_shannon_index, height_kurtosis, sigma_z, height_skewness, height_stddev, 
            height_variance, pt_veg_num);

    }
    
    
    return 0;
}
