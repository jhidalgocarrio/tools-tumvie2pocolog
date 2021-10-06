/* Generated from orogen/lib/orogen/templates/tasks/Task.hpp */

#ifndef TUMVIE2POCOLOG_TASK_TASK_HPP
#define TUMVIE2POCOLOG_TASK_TASK_HPP

#include <json/json.h>
#include <json/value.h>
#include <fstream>

#include <opencv2/core/mat.hpp>
#include <opencv2/calib3d.hpp>

#include <base/samples/Frame.hpp>
#include "tumvie2pocolog/TaskBase.hpp"

namespace tumvie2pocolog{

    struct CameraCalib
    {
        cv::Mat K; // intrinsics
        cv::Mat Kr; //rectified K
        cv::Vec4d D; //distortion
        cv::Mat Rr;// rect matrix
        std::string distortion_model; //model
        int height, width; //image size
        base::Transform3d T_imu_cam; //Transformation matrix in TUM-Vie T_imu_cam
    };

    struct Event
    {
        /** Variable **/
        std::vector<double> t;
        std::vector<double> x;
        std::vector<double> y;
        std::vector<double> p;
    };

     /*! \class Task
     * \brief The task context provides and requires services. It uses an ExecutionEngine to perform its functions.
     * Essential interfaces are operations, data flow ports and properties. These interfaces have been defined using the oroGen specification.
     * In order to modify the interfaces you should (re)use oroGen and rely on the associated workflow.
     * Declare a new task context (i.e., a component)

The corresponding C++ class can be edited in tasks/Task.hpp and
tasks/Task.cpp, and will be put in the tumvie2pocolog namespace.
     * \details
     * The name of a TaskContext is primarily defined via:
     \verbatim
     deployment 'deployment_name'
         task('custom_task_name','tumvie2pocolog::Task')
     end
     \endverbatim
     *  It can be dynamically adapted when the deployment is called with a prefix argument.
     */
    class Task : public TaskBase
    {
    friend class TaskBase;
    protected:

        /** Mean gravity value at Earth surface [m/s^2] **/
        static constexpr float GRAVITY = 9.81;

        /** Comfiguration **/
        tumvie2pocolog::Config config;

        /** Variable **/
        tumvie2pocolog::Event events;
        std::vector<long double> image_ts;
        std::vector<std::string> img_fname;

        /** Calibration information **/
        CameraCalib event_cam_calib;
        CameraCalib rgb_cam_calib;

        /** Output ports **/
        RTT::extras::ReadOnlyPointer<base::samples::frame::Frame> img_msg;

    protected:

        void readH5Dataset(std::string fname, std::string dataset, std::vector<double> &data);

        void writeEvents(const float &t_offset);

        bool writeIMU(const std::string &fname);

        void writeRGB();

        cv::Mat RGBToEventFrame(cv::Mat &frame,  cv::Mat &P, int &height, int &width);

    public:
        /** TaskContext constructor for Task
         * \param name Name of the task. This name needs to be unique to make it identifiable via nameservices.
         * \param initial_state The initial TaskState of the TaskContext. Default is Stopped state.
         */
        Task(std::string const& name = "tumvie2pocolog::Task");

        /** Default deconstructor of Task
         */
        ~Task();

        /** This hook is called by Orocos when the state machine transitions
         * from PreOperational to Stopped. If it returns false, then the
         * component will stay in PreOperational. Otherwise, it goes into
         * Stopped.
         *
         * It is meaningful only if the #needs_configuration has been specified
         * in the task context definition with (for example):
         \verbatim
         task_context "TaskName" do
           needs_configuration
           ...
         end
         \endverbatim
         */
        bool configureHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to Running. If it returns false, then the component will
         * stay in Stopped. Otherwise, it goes into Running and updateHook()
         * will be called.
         */
        bool startHook();

        /** This hook is called by Orocos when the component is in the Running
         * state, at each activity step. Here, the activity gives the "ticks"
         * when the hook should be called.
         *
         * The error(), exception() and fatal() calls, when called in this hook,
         * allow to get into the associated RunTimeError, Exception and
         * FatalError states.
         *
          std::cout<<"CALIB EVENT CAM:"<<this->config.event_camera_idx<<std::endl;
    std::cout<<"Model:"<<this->event_cam_calib.distortion_model<<std::endl;
    std::cout<<"Height:"<<this->event_cam_calib.height<<std::endl;
    std::cout<<"Width:"<<this->event_cam_calib.width<<std::endl;
    std::cout<<"K:"<<this->event_cam_calib.K<<std::endl;
    std::cout<<"D:"<<this->event_cam_calib.D<<std::endl;
    std::cout<<"Kr:"<<this->event_cam_calib.Kr<<std::endl;
    std::cout<<"Rr:"<<this->event_cam_calib.Rr<<std::endl;
    std::cout<<"Q:"<<this->event_cam_calib.Q<<std::endl;
    std::cout<<"T:"<<this->event_cam_calib.Tij<<std::endl;

   * In the first case, updateHook() is still called, and recover() allows
         * you to go back into the Running state.  In the second case, the
         * errorHook() will be called instead of updateHook(). In Exception, the
         * component is stopped and recover() needs to be called before starting
         * it again. Finally, FatalError cannot be recovered.
         */
        void updateHook();

        /** This hook is called by Orocos when the component is in the
         * RunTimeError state, at each activity step. See the discussion in
         * updateHook() about triggering options.
         *
         * Call recover() to go back in the Runtime state.
         */
        void errorHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Running to Stopped after stop() has been called.
         */
        void stopHook();

        /** This hook is called by Orocos when the state machine transitions
         * from Stopped to PreOperational, requiring the call to configureHook()
         * before calling start() again.
         */
        void cleanupHook();

        /** Read JSON TUM-VIE calibration file **/
        static CameraCalib readCameraInfo(std::string calib_fname, std::string cam_id)
        {
            CameraCalib calib;
            Json::Value calib_data;
            std::ifstream calib_file(calib_fname, std::ifstream::binary);
            calib_file >> calib_data;

            auto intrinsics_parser = [&calib] (Json::Value &data)
            {
                calib.K = cv::Mat_<double>::eye(3, 3);
                calib.K.at<double>(0,0) = data["fx"].asDouble();
                calib.K.at<double>(1,1) = data["fy"].asDouble();
                calib.K.at<double>(0,2) = data["cx"].asDouble();
                calib.K.at<double>(1,2) = data["cy"].asDouble();
            };
            auto distort_coeff_parser = [&calib] (Json::Value &data)
            {
                calib.D = cv::Vec4d(data["k1"].asDouble(),
                                    data["k2"].asDouble(),
                                    data["k3"].asDouble(),
                                    data["k4"].asDouble());
            };
            auto extrinsics_parser = [&calib] (Json::Value &data)
            {
                Eigen::Vector3d trans(data["px"].asDouble(), data["py"].asDouble(), data["pz"].asDouble());
                Eigen::Quaterniond q(data["qw"].asDouble(), data["qx"].asDouble(), data["qy"].asDouble(),
                                     data["qz"].asDouble());
                calib.T_imu_cam = base::Transform3d(q);
                calib.T_imu_cam.translation() = trans;
            };

            Json::Value resolution = calib_data["value0"]["resolution"][std::stoi(cam_id)];
            calib.width = resolution[0].asInt();
            calib.height = resolution[1].asInt();

            Json::Value camera_info = calib_data["value0"]["intrinsics"][std::stoi(cam_id)];
            intrinsics_parser(camera_info["intrinsics"]);
            distort_coeff_parser(camera_info["intrinsics"]);
            calib.distortion_model = camera_info["camera_type"].asString();

            Json::Value extrinsics = calib_data["value0"]["T_imu_cam"][std::stoi(cam_id)];
            extrinsics_parser(extrinsics);

            calib.Rr = cv::Mat_<double>::eye(3, 3); //Identity

            cv::fisheye::estimateNewCameraMatrixForUndistortRectify(calib.K, calib.D, cv::Size(calib.width, calib.height), calib.Rr, calib.Kr);

            return calib;
        }

        template <typename P, typename V > 
        static void drawEventsOnImage (const std::vector<P> &points, const std::vector<V> &values, cv::Mat &img,
                                    cv::Vec3b &color_positive, cv::Vec3b &color_negative, const std::string &method = "nn")
        {
            auto clip = [](const int n, const int lower, const int upper)
            {
                return std::max(lower, std::min(n, upper));
            };

            cv::Size s = img.size();
            auto it_x = points.begin();
            auto it_p = values.begin();
            while(it_x != points.end() && it_p != values.end())
            {
                /** Get the color based on the polarity **/
                auto color = color_negative;
                if ((*it_p))
                {
                    auto color = color_positive;
                }

                if (method.compare("nn") == 0)
                {
                    cv::Point2i x_int = *it_x;
                    x_int.x = clip(x_int.x, 0, s.width - 1);
                    x_int.y = clip(x_int.x, 0, s.height - 1);

                    img.at<cv::Vec3b>(x_int) = color;
                    
                }
                else if (method.compare("bilinear"))
                {
                    int x0 = floor(it_x->x);
                    int y0 = floor(it_x->y);
                    int x1 = x0 + 1;
                    int y1 = x1 + 1;

                    /** compute the voting weights. Note: assign weight 0 if the point is out of the image **/
                    float wa = (x1 - it_x->x) * (y1 - it_x->y) * (x0 < s.width) * (y0 < s.height) * (x0 >= 0) * (y0 >= 0);
                    float wb = (x1 - it_x->x) * (it_x->y - y0) * (x0 < s.width) * (y1 < s.height) * (x0 >= 0) * (y1 >= 0);
                    float wc = (it_x->x - x0) * (y1 - it_x->y) * (x1 < s.width) * (y0 < s.height) * (x1 >= 0) * (y0 >= 0);
                    float wd = (it_x->x - x0) * (it_x->y - y0) * (x1 < s.width) * (y1 < s.height) * (x1 >= 0) * (y1 >= 0);

                    x0 = clip(x0, 0, s.width - 1);
                    x1 = clip(x1, 0, s.width - 1);
                    y0 = clip(y0, 0, s.height - 1);
                    y1 = clip(y1, 0, s.height - 1);
 
                    img.at<cv::Vec3b>(x0, y0) = color;
                    img.at<cv::Vec3b>(x0, y1) = color;
                    img.at<cv::Vec3b>(x1, y0) = color;
                    img.at<cv::Vec3b>(x1, y1) = color;
                }
                ++it_x;
                ++it_p;
            }
            return;
        }

        static std::string type2str(int type)
        {
            std::string r;

            uchar depth = type & CV_MAT_DEPTH_MASK;
            uchar chans = 1 + (type >> CV_CN_SHIFT);

            switch ( depth ) {
                case CV_8U:  r = "8U"; break;
                case CV_8S:  r = "8S"; break;
                case CV_16U: r = "16U"; break;
                case CV_16S: r = "16S"; break;
                case CV_32S: r = "32S"; break;
                case CV_32F: r = "32F"; break;
                case CV_64F: r = "64F"; break;
                default:     r = "User"; break;
            }

            r += "C";
            r += (chans+'0');

            return r;
        };
    };
}

#endif

