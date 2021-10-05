#ifndef tumvie2pocolog_TYPES_HPP
#define tumvie2pocolog_TYPES_HPP

/* If you need to define types specific to your oroGen components, define them
 * here. Required headers must be included explicitly
 *
 * However, it is common that you will only import types from your library, in
 * which case you do not need this file
 */

#include <string>
#include <base/samples/Frame.hpp>

namespace tumvie2pocolog
{
    struct Config
    {
        std::string event_camera_idx; //index of the camera in the calib [0, 1, 2, 3]
        std::string rgb_camera_idx;  //index of the camera in the calib [0, 1, 2, 3]

        unsigned int events_pkgsize; //number of events in event array message

        /** Path to folders, all path folders are relative to the root folder **/
        std::string root_folder; // path root folder to the dataset
        std::string output_folder; // path to output folder
        std::string images_folder; //folder where the images are

        /** File name, all path files are relative to the root folder **/
        std::string img_ts_filename; //file with timestamps
        std::string events_filename; //HDF5 file for events
        std::string imu_filename; //HDF5 file for imu
        std::string calib_filename; //calibration file
    };

    /** Color encoding: first color positive event, seconds color negative event **/
    enum COLOR_ENCODING{BLUE_RED, GREEN_RED, BLUE_BLACK};

}

#endif

