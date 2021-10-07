/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include <iostream>
#include <iomanip>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/calib3d.hpp>

#include <glob.h>

#include <blosc_filter.h>
#include <hdf5/serial/hdf5.h>
#include <hdf5/serial/H5Cpp.h>
#include <boost/filesystem.hpp>

/** Base types **/
#include <base/samples/IMUSensors.hpp>
#include <base/samples/EventArray.hpp>

/** Frame helper **/
#include <frame_helper/FrameHelper.h>

#include "Task.hpp"

using namespace H5;
using namespace tumvie2pocolog;
namespace fs = boost::filesystem;

Task::Task(std::string const& name)
    : TaskBase(name)
{

}

Task::~Task()
{

}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool Task::configureHook()
{
    if (! TaskBase::configureHook())
        return false;

    /** Register BLOSC filter **/
    char *version, *date;
    int r = register_blosc(&version, &date);
    printf("Blosc version info: %s (%s) (%d)\n", version, date, r);

    /** Configuration **/
    this->config = _config.value();

    /** Read the calibration file **/
    fs::path calib_fname = fs::path(config.root_folder)/ fs::path(config.calib_filename);
    std::cout<<"Calib file: "<<calib_fname<<std::endl;
    this->event_cam_calib = Task::readCameraInfo(calib_fname.string(), this->config.event_camera_idx);
    this->rgb_cam_calib = Task::readCameraInfo(calib_fname.string(), this->config.rgb_camera_idx);

    /** Get the T_cam_event_cam_rgb transformation **/
    Eigen::Matrix4d T_event_rgb = (this->event_cam_calib.T_imu_cam.inverse() * this->rgb_cam_calib.T_imu_cam).matrix();
    cv::Mat T; cv::eigen2cv(T_event_rgb, T);
    std::cout<<"T_event_rgb:\n"<<T<<std::endl;

    /** Compute the projection matrix for the Event camera **/
    cv::Mat R = cv::Mat_<double>::eye(3, 3); cv::Mat t = cv::Mat_<double>::zeros(3, 1);//rows x cols
    //std::cout<<"R:\n"<<R<<"\nt:\n"<<t<<std::endl;
    cv::hconcat(this->event_cam_calib.K*R, this->event_cam_calib.K*t, this->event_cam_calib.P);

    /** Compute the projection matrix for the RGB camera **/
    R = T(cv::Rect(0,0,3,3)).clone(); t = T(cv::Rect(3,0,1,3)).clone();
    //std::cout<<"R:\n"<<R<<"\nt:\n"<<t<<std::endl;
    cv::hconcat(this->event_cam_calib.K*R, this->event_cam_calib.K*t, this->rgb_cam_calib.P);

    /** Get the mapping functions for event camera**/
    cv::fisheye::initUndistortRectifyMap(this->event_cam_calib.K, this->event_cam_calib.D,
                            cv::Mat(), this->event_cam_calib.P,
                            cv::Size(this->event_cam_calib.width, this->event_cam_calib.height),
                            CV_32FC1, this->event_cam_calib.mapx, this->event_cam_calib.mapy);

    /** Get the mapping functions for rgb camera**/
    cv::fisheye::initUndistortRectifyMap(this->rgb_cam_calib.K, this->rgb_cam_calib.D,
                            cv::Mat(), this->rgb_cam_calib.P,
                            cv::Size(this->event_cam_calib.width, this->event_cam_calib.height),
                            CV_32FC1, this->rgb_cam_calib.mapx, this->rgb_cam_calib.mapy);

    std::cout<<"CALIB EVENT CAM:"<<this->config.event_camera_idx<<std::endl;
    std::cout<<"Model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->event_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->event_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"D:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"P:"<<this->event_cam_calib.P<<std::endl;
    std::cout<<"mapx: "<<this->event_cam_calib.mapx.rows<<" x "<<this->event_cam_calib.mapx.cols<<std::endl;
    std::cout<<"mapy: "<<this->event_cam_calib.mapy.rows<<" x "<<this->event_cam_calib.mapy.cols<<std::endl;
    std::cout<<"T_imu_cam:"<<this->event_cam_calib.T_imu_cam.matrix()<<std::endl;

    std::cout<<"CALIB RGB CAM:"<<this->config.rgb_camera_idx<<std::endl;
    std::cout<<"Model:"<<this->rgb_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->rgb_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->rgb_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->rgb_cam_calib.K<<std::endl;
    std::cout<<"D:"<<this->rgb_cam_calib.D<<std::endl;
    std::cout<<"P:"<<this->rgb_cam_calib.P<<std::endl;
    std::cout<<"mapx: "<<this->rgb_cam_calib.mapx.rows<<" x "<<this->rgb_cam_calib.mapx.cols<<std::endl;
    std::cout<<"mapy: "<<this->rgb_cam_calib.mapy.rows<<" x "<<this->rgb_cam_calib.mapy.cols<<std::endl;
    std::cout<<"T_imu_cam:"<<this->rgb_cam_calib.T_imu_cam.matrix()<<std::endl;

    /** Read images timestamps **/
    fs::path img_ts_fname = fs::path(config.root_folder)/ fs::path(config.img_ts_filename);
    std::cout<<"Images file: "<<img_ts_fname<<std::endl;
    std::ifstream infile;
    infile.open(img_ts_fname.string());
    if (!infile)
    {
        std::cout << "Unable to open file:"<<img_ts_fname.string()<<std::endl;
        return false; // terminate with error
    }

    std::string line;
    while (!infile.eof())
    {
        getline(infile, line);
        if (line[0] != '#' and line[0] != '\n' and line[0] != '\0')
        {
            this->image_ts.push_back(std::stold(line));
            //std::cout<<std::stold(line)<<std::endl;
        }
    }
    infile.close();

    /** Name for the all the images **/
    fs::path path_images = fs::path(config.root_folder)/ fs::path(config.images_folder)/fs::path("*.jpg");
    std::cout<<path_images.string()<<std::endl;
    glob::glob glob_img(path_images.string());
    while (glob_img)
    {
        std::string path = (fs::path(config.root_folder)/fs::path(config.images_folder)/fs::path(glob_img.current_match())).string();
        this->img_fname.push_back(path);
        glob_img.next();
    }
    std::sort(this->img_fname.begin(), this->img_fname.end());

    if (this->image_ts.size() != this->img_fname.size())
    {
        RTT::log(RTT::Error) << "[ERROR]: Number of images and timestamps does not match"<< RTT::endlog();
        return false;
    }

    /** Read Events data **/
    fs::path events_fname = fs::path(config.root_folder)/ fs::path(config.events_filename);
    std::cout<<"Events file: "<<events_fname<<std::endl;
    try
    {
        std::vector<double> data;
        readH5Dataset(events_fname.string(), "events/t", data);
        this->events.t = std::vector<double> (data);
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/x", data);
        this->events.x = std::vector<double> (data.begin(), data.end());
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/y", data);
        this->events.y = std::vector<double> (data.begin(), data.end());
        data.resize(0);

        readH5Dataset(events_fname.string(), "events/p", data);
        this->events.p = std::vector<double> (data.begin(), data.end());
        data.resize(0);
    }
    // catch failure caused by the H5File operations
    catch( FileIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSet operations
    catch( DataSetIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataSpaceIException error )
    {
        error.printErrorStack();
        return -1;
    }
    // catch failure caused by the DataSpace operations
    catch( DataTypeIException error )
    {
        error.printErrorStack();
        return -1;
    }

    /** Set the img member **/
    ::base::samples::frame::Frame *img = new ::base::samples::frame::Frame();
    this->img_msg.reset(img);
    img = nullptr;

    return true;
}
bool Task::startHook()
{
    if (! TaskBase::startHook())
        return false;

    /** Write the Events **/
    this->writeEvents(0.0);

    /** Write IMU values **/
    fs::path imu_fname = fs::path(config.root_folder)/ fs::path(config.imu_filename);
    this->writeIMU(imu_fname.string());

    /** Write the images **/
    this->writeRGB();

    return true;
}
void Task::updateHook()
{
    TaskBase::updateHook();
}
void Task::errorHook()
{
    TaskBase::errorHook();
}
void Task::stopHook()
{
    TaskBase::stopHook();
}
void Task::cleanupHook()
{
    TaskBase::cleanupHook();
}

void Task::readH5Dataset(std::string fname, std::string dataset, std::vector<double> &data)
{
    H5File file( fname.c_str(), H5F_ACC_RDONLY );
    DataSet dset = file.openDataSet(dataset.c_str());
    std::cout<<"Reading dataset "<<dataset<<std::endl;

    //Get dataspace of the dataset.
    DataSpace dataspace = dset.getSpace();

    // Get the number of dimensions in the dataspace.
    int rank = dataspace.getSimpleExtentNdims();

    if (rank == 0)
    {
        // for single value datasets
        // create a vector the same size as the dataset
        data.resize(1);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;
        dset.read(data.data(), PredType::NATIVE_DOUBLE, H5S_ALL, H5S_ALL);
    
    }
    else if (rank == 1)
    {
        // for array datasets
        // Get the dimension size of each dimension in the dataspace and display them.
        hsize_t dims_out[1];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        std::cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) << std::endl;
        std::cout<<"ndims :"<<ndims<<std::endl;
        // Define the memory dataspace
        DataSpace memspace (1,dims_out);

        // create a vector the same size as the dataset
        data.resize(dims_out[0]);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;

        // pass pointer to the array (or vector) to read function, along with the data type and space.
        dset.read(data.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);
    }
    else if (rank == 2)
    {
        hsize_t dims_out[2];
        int ndims = dataspace.getSimpleExtentDims( dims_out, NULL);
        std::cout << "rank " << rank << ", dimensions " <<
                (unsigned long)(dims_out[0]) <<" x " <<(unsigned long) (dims_out[1])<< std::endl;
        std::cout<<"ndims :"<<ndims<<std::endl;
        // Define the memory dataspace
        DataSpace memspace (rank,dims_out);

        // create a vector the same size as the dataset
        data.resize(dims_out[0]*dims_out[1]);
        std::cout<<"Vectsize: "<<data.size()<<std::endl;

        // pass pointer to the array (or vector) to read function, along with the data type and space.
        dset.read(data.data(), PredType::NATIVE_DOUBLE, memspace, dataspace);
    }

    // close the HDF5 file
    file.close();
}

void Task::writeEvents(const float &t_offset)
{
    /** Write the Events **/
    ::base::samples::EventArray events_msg;
    std::cout<<"Writing Events... ";
    for (size_t i=0; i<this->events.t.size(); ++i)
    {
        ::base::samples::Event ev(
            static_cast<uint16_t>(this->events.x[i]), static_cast<uint16_t>(this->events.y[i]),
            ::base::Time::fromMicroseconds(static_cast<int64_t>(this->events.t[i] + t_offset)),
            (uint8_t)this->events.p[i]);

        if (events_msg.events.size() == 0)
        {
            events_msg.time = ev.ts;
        }
        events_msg.events.push_back(ev);

        if (i%this->config.events_pkgsize == 0)
        {
            //std::cout<<"events ["<<events_msg.events.size()<<"] at"<<events_msg.time.toString()<<std::endl;
            events_msg.height = this->event_cam_calib.height;
            events_msg.width = this->event_cam_calib.width;
            RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
            while (status != RTT::WriteStatus::WriteSuccess)
                status = this->_events.write(events_msg);
            events_msg.events.clear();
        }

    }

    /** Write the last event array **/
    if (events_msg.events.size() > 0)
    {
        std::cout<<"last ["<<events_msg.events.size()<<"] ";
        events_msg.height = this->event_cam_calib.height;
        events_msg.width = this->event_cam_calib.width;
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = this->_events.write(events_msg);
    }
 
    std::cout<<"[DONE]"<<std::endl;
}

bool Task::writeIMU(const std::string &fname)
{
    /** Read and Write IMU data **/
    std::cout<<"IMU file: "<<fname<<std::endl;
    std::ifstream infile;
    infile.open(fname);
    if (!infile)
    {
        std::cout << "Unable to open file:"<<fname<<std::endl;
        return false; // terminate with error
    }

    /** Loop until end of file **/
    std::cout<<"Writing IMU... ";
    std::string line;
    while (!infile.eof())
    {
        getline(infile, line);
        if (line[0] != '#' and line[0] != '\n' and line[0] != '\0')
        {
            size_t pos = 0;
            std::vector<long double> values;
            std::string space_delimiter = " ";
            while ((pos = line.find(space_delimiter)) != std::string::npos)
            {
                values.push_back(std::stold(line.substr(0, pos)));
                line.erase(0, pos + space_delimiter.length());
            }
            ::base::samples::IMUSensors imusamples;
            imusamples.time = ::base::Time::fromMicroseconds(static_cast<int64_t>(values[0]));
            //std::cout<<"IMU time: "<<imusamples.time.toSeconds()<<std::endl;
            imusamples.gyro << values[1], values[2], values[3]; //[rad/s]
            imusamples.acc << values[4], values[5], values[6]; //[m/s^2]
            RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
            while (status != RTT::WriteStatus::WriteSuccess)
                status = this->_imu.write(imusamples);
        }
    }
    infile.close();
    std::cout<<"[DONE]"<<std::endl;
    return true;
}

void Task::writeRGB()
{
    /** Write the images **/
    auto it_img =this->img_fname.begin();
    auto it_ts =this->image_ts.begin();
    std::cout<<"Writing images... ";
    while(it_img != this->img_fname.end() && it_ts != this->image_ts.end())
    {
        /** Read the image file **/
        cv::Mat orig_img = cv::imread(*it_img, cv::IMREAD_COLOR);

        /** Remap RGB image in the event camera frame **/
        cv::Mat img; remap(orig_img, img, this->rgb_cam_calib.mapx, this->rgb_cam_calib.mapy, cv::INTER_CUBIC);

        /** Convert from cv mat to frame **/
        ::base::samples::frame::Frame *img_msg_ptr = this->img_msg.write_access();
        img_msg_ptr->image.clear();
        frame_helper::FrameHelper::copyMatToFrame(img, *img_msg_ptr);

        /** Write into the port **/
        img_msg_ptr->time = ::base::Time::fromMicroseconds(*it_ts);
        img_msg_ptr->received_time = img_msg_ptr->time;
        this->img_msg.reset(img_msg_ptr);
        RTT::WriteStatus status = RTT::WriteStatus::WriteFailure;
        while (status != RTT::WriteStatus::WriteSuccess)
            status = _frame.write(this->img_msg);

        ++it_img;
        ++it_ts;
    }
    std::cout<<"[DONE]"<<std::endl;
}

cv::Mat Task::RGBToEventFrame(cv::Mat &frame,  cv::Mat &P, int &height, int &width)
{
    cv::Mat out_img (cv::Size(width, height), CV_8UC3, cv::Scalar(0, 0, 0));
    //std::cout<<"Out image "<<out_img.size()<<" TYPE: "<<type2str(out_img.type())<<std::endl;

    /** RGB image in event camera (backward warping ) **/
    cv::Mat map (height, width, CV_32FC2); //event frame size -> rgb camera size
    std::cout<<"Original Image size: "<<frame.size()<<std::endl;
    for (int y=0; y<height; ++y)
    {
        for(int x=0; x<width; ++x)
        {
            cv::Point3d u_hom(x, y, 1.0);
            cv::Mat_<double> u_hat = P * cv::Mat(u_hom, false);
            //cv::Vec3b value = frame.at<cv::Vec3b>(floor(u_hom.y), floor(u_hom.x));
            //out_img.at<cv::Vec3b>(cv::Point(x, y)) = value;
            map.at<cv::Point2f>(y, x) = cv::Point2f(u_hat(0, 0), u_hat(0, 1));
        }
    }
    //std::cout<<"Map size: "<<map.size()<<std::endl;
    cv::remap(frame, out_img, map, cv::Mat(), cv::INTER_LINEAR);
    std::cout<<"Out image "<<out_img.size()<<" TYPE: "<<type2str(out_img.type())<<std::endl;

    return out_img;
}