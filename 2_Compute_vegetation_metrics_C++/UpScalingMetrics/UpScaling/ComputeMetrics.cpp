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

/// <summary>
/// Select a random number of elements in the container.
/// </summary>
/// <param name="vec"></param> The container in which stores the entire elements.
/// <param name="count"></param> The number of elements that to be retrieved.
/// <returns></returns> The elements that are randomly selected.
std::vector<unsigned> selectRandomElements(const std::vector<unsigned>& vec, size_t count)
{
    std::vector<unsigned> result(vec);
    std::random_device rd;
    std::mt19937 eng(rd());

    // Shuffle the vector;
    std::shuffle(result.begin(), result.end(), eng);

    // Resize the vector to contain only "count" elements;
    if (count < result.size())
    {
        result.resize(count);
    }

    return result;
}

/// <summary>
/// Generate a random number of vectors.
/// </summary>
/// <param name="count"></param> The number of vectors
/// <returns></returns> The obtained vectors.
std::vector<unsigned> generateVector(unsigned count)
{
    std::vector<unsigned> indices(count);
    int current = 1;

    std::generate(indices.begin(), indices.end(), [&current]() {return current++; });

    return indices;
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

double calculateSkewness_1(const std::vector<double>& data)
{
    int n = data.size();
    if (n < 3) 
    {
        std::cout << "Need at least 3 data points to calculate skewness.\n";
        return std::numeric_limits<double>::quiet_NaN();
    }

    // Calculate mean
    double mean = std::accumulate(data.begin(), data.end(), 0.0) / n;

    // Calculate standard deviation
    double sq_sum = 0.0;
    for (double value : data) 
    {
        sq_sum += std::pow(value - mean, 2);
    }
    double stdev = std::sqrt(sq_sum / n);

    // Calculate skewness
    double skewness = 0.0;
    for (double value : data) 
    {
        skewness += std::pow((value - mean) / stdev, 3);
    }
    skewness *= static_cast<double>(n) / ((n - 1) * (n - 2));

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
/// Main function for the calculation of the 25 metrics using point clouds
/// </summary>
/// <param name="argc"></param>
/// <param name="argv"></param>
/// <returns></returns>
int main(int argc, char** argv)
{
    const float area = std::pow(2, 2);
    const int pts_num_per_mm = 20;
    const float ratio = area * pts_num_per_mm;

    std::string path = "D:\\TestData\\Upscaling\\Output\\Plotting\\2m\\Grassland_2m\\";
    std::string inPath = path + "\\plots\\";
    std::string metricPath = path + "metrics_density\\";
    createDirectory(metricPath);

    //std::string outPath = path + "down_density_" + std::to_string(pts_num_per_mm) + "\\";
    std::string outPath = path + "down_density_original\\";
    createDirectory(outPath);
    std::string outPathAll = outPath + "All\\";
    createDirectory(outPathAll);

    std::string outMetrics = metricPath + "metrics_density_original.txt"; 
  
    FILE* outFileMetrics = fopen(outMetrics.c_str(), "w");

    std::vector<std::string> files;
    getAllFiles(inPath, files);
    std::cout << "There are: " << files.size() << " files in the folder." << std::endl;

    for (auto& file : files)
    {
        std::vector<dPoint3D> GroundPts;
        std::vector<dPoint3D> VegetationPts;
        std::vector<dPoint3D> NormalizedVegetationPts;


        LASreadOpener readOpener;
        readOpener.set_file_name(file.c_str());
        LASreader* reader = readOpener.open();

        fs::path pathObj(file);
        std::string fileName = pathObj.stem().string();

        CLazPointData* cloud = new CLazPointData();

        if (cloud && cloud->LoadFile(file.c_str()))
        {
            size_t numPtAll = cloud->m_PtsNum;

            size_t numPtGround = cloud->m_GroundPt.size();
            size_t numPtVegetation = cloud->m_VegetaPt.size();

            // Total number of points;
            std::vector<unsigned> indicesAll = generateVector(numPtAll);
            std::vector<unsigned> indicesGround = generateVector(numPtGround);
            std::vector<unsigned> indicesVegetation = generateVector(numPtVegetation);

            std::vector<unsigned> dGround = selectRandomElements(cloud->m_GroundPt, cloud->m_GroundPt.size());
            std::vector<unsigned> dVege = selectRandomElements(cloud->m_VegetaPt, cloud->m_VegetaPt.size());

            std::string outAllPtsPath = outPathAll + fileName + "_all.laz";

            LASwriteOpener openerAll;
            openerAll.set_file_name(outAllPtsPath.c_str());
            LASwriter* writerAll = openerAll.open(&reader->header);


            // All points
            writerAll->update_header(&reader->header, true);
            writerAll->close();
            delete writerAll;
            writerAll = nullptr;
        }

        if (VegetationPts.empty() || GroundPts.empty())
        {
            continue;
        }
        

        PointCloud2<double> kdPts;
        kdPts.pts.resize(GroundPts.size());
        int index = 0;
        for (auto& pt : GroundPts)
        {
            kdPts.pts[index].x = pt.x;
            kdPts.pts[index].y = pt.y;
            kdPts.pts[index].z = 0.0;
            index++;
        }
        kdTree* tree = new kdTree(3, kdPts, KDTreeSingleIndexAdaptorParams(10));
        tree->buildIndex();

        const size_t ret_num = 1;
        std::vector<size_t> ret_index(ret_num);
        std::vector<double> out_dist_sqr(ret_num);


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
        float penetration_ratio = GroundPts.size() * 1.0 / (GroundPts.size() * 1.0 + VegetationPts.size() * 1.0);
        float height_max = maxHeight;
        float height_mean = heightSum;
        float height_median = findMedian(heights);
        float height_stddev = calculateStandardDeviation(heights);
        float height_shannon_index = calculateShannonIndex(heights);
        float height_skewness = calculateSkewness(heights);
        float height_kurtosis = calculateKurtosis(heights);
        float height_25 = calculate_Hp25(heights);
        float height_50 = calculate_Hp50(heights);
        float height_75 = calculate_Hp75(heights);
        float height_95 = calculate_Hp95(heights);

        int density_above_mean_z = calculate_density_above_mean_z(heights, heightSum);

        std::vector<double> brs;
        calculate_br(heights, brs);

        float coeff_var_z = height_stddev / height_mean;

        float height_variance = std::pow(height_stddev, 2);
        float sigma_z = calculate_sigma_z(heights, height_mean);


        fprintf(outFileMetrics, "%lf %lf %lf %lf %lf %lf %lf %lf %d %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            height_max, height_mean, height_median, height_25, height_50, height_75, height_95,
            penetration_ratio, density_above_mean_z,
            brs[0], brs[1], brs[2], brs[3], brs[4], brs[5], brs[6], brs[7], brs[8],
            coeff_var_z, height_shannon_index, height_kurtosis, sigma_z, height_skewness, height_stddev,
            height_variance);

        if (cloud)
        {
            delete cloud;
            cloud = nullptr;
        }
        reader->close();
        delete reader;
        reader = nullptr;

        if (tree)
        {
            delete tree;
            tree = nullptr;
        }
    }

    fclose(outFileMetrics);
    return 0;
}
