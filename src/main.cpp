/*
 * CS5330 Project 4 - Calibration and Augmented Reality
 * Author - Amit Mulay
 */

#include <opencv2/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <iostream>
#include "OBJParser.h"

int storeCameraConfig(cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    cv::FileStorage fs("CameraConfig.yaml", cv::FileStorage::WRITE);
    fs << "intriMat" << cameraMatrix;
    fs << "distCoeff" << distCoeffs;
    fs.release();
    return 0;
}

int readCameraConfig(cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
{
    cv::FileStorage fs("CameraConfig.yaml", cv::FileStorage::READ);
    fs ["intriMat"] >> cameraMatrix;
    fs ["distCoeff"] >> distCoeffs;
    std::cout << "CameraMatrix" << cameraMatrix << std::endl;
    std::cout << "distCoeff" << distCoeffs << std::endl;
    fs.release();
    return 0;
}

int main(int argc, char *argv[])
{
    cv::VideoCapture *capdev;
    
    std::vector< cv::Point2f > corner_set;
    std::vector< std::vector< cv::Point2f > > corner_list;
    std::vector<cv::Point3f> point_set;
    std::vector< std::vector< cv::Point3f > > point_list;
    
    // open the video device
    capdev = new cv::VideoCapture(0);
    if( !capdev->isOpened() ) {
            printf("Unable to open video device\n");
            return(-1);
    }

    // get some properties of the image
    cv::Size refS( (int) capdev->get(cv::CAP_PROP_FRAME_WIDTH ),
                   (int) capdev->get(cv::CAP_PROP_FRAME_HEIGHT));
    
    cv::namedWindow("Video", 1); // identifies a window
    cv::namedWindow("Gray", 2); // identifies a window
    cv::Mat frame;
    cv::Mat gray;
    
    
    
    *capdev >> frame;
    
    int board_width = 9;
    int board_height = 6;
    float square_side = 0.18;
    
    cv::Size board_size = cv::Size(board_width, board_height);
    int board_n = board_width * board_height;
    cv::Mat cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat distCoeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat rvecs;
    cv::Mat tvecs;
    bool displayPose = false;
    bool displayFeatures = false;
    
    std::string bunny_path = cv::samples::findFile("../objects/house_obj.obj");
    OBJParser op;
    op.parseFile(bunny_path);
    int scNum = 1;
    for(;;)
    {
        
        char key = cv::waitKey(30);
        
        *capdev >> frame; // get a new frame from the camera, treat as a stream
        if( frame.empty() )
        {
          printf("frame is empty\n");
          break;
        }
        cv::resize(frame, frame, cv::Size(), 0.50, 0.50);
        
        cv::cvtColor(frame, gray, cv::COLOR_RGBA2GRAY, 0);
        
        corner_set.clear();
        point_set.clear();
        
        bool found = false;
        
        found = cv::findChessboardCorners(frame, board_size, corner_set,
                                              cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);
        if (found)
        {
            //If corners are detected, capture coordinate information.
            cv::cornerSubPix(gray, corner_set, cv::Size(5, 5), cv::Size(-1, -1),
                       cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
            cv::drawChessboardCorners(gray, board_size, corner_set, found);
            for (int i = -2; i <= board_height/2; i++)
            {
                for (int j = -4; j <= board_width/2; j++)
                {
                    point_set.push_back(cv::Point3f((float)j * square_side, (float)i * square_side, 0));
                }
            }
        }
        
        
        
        if( key == 's')
        {
            //Build point set
            if (!found)
            {
                std::cout << "Corners not found!" << std::endl;
                
            }
            else{
                std::cout << "Corners found and parameters saved!" << std::endl;
                
                //Store image
                cv::imwrite("screencapture_" + std::to_string(scNum) + ".jpg", gray);
                scNum++;
                //store corner set and point set in list
                corner_list.push_back(corner_set);
                point_list.push_back(point_set);
                
                //Allow caliberation if list contains atleast 5 corner sets and point sets
                if(corner_list.size() > 5)
                {
                    cameraMatrix.at<double>(0,2) = frame.cols/2;
                    cameraMatrix.at<double>(1,2) = frame.rows/2;
                    std::vector<cv::Mat> rvecs, tvecs;
                    
                    double rms = cv::calibrateCamera(point_list, corner_list, frame.size(), cameraMatrix, distCoeffs, rvecs, tvecs, cv::CALIB_FIX_ASPECT_RATIO,cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 30, 0.1));
                    
                    std::cout << "Camera Matrix:" << std::endl << cameraMatrix << std::endl;
                    
                    std::cout << "Distortion Coeff:" << distCoeffs << std::endl;
                    
                    std::cout << "Reprojection error:" << rms << std::endl;
                    
                    std::cout << "Corners list" << std::endl;
                    storeCameraConfig(cameraMatrix,distCoeffs);
                    
                }
            }
            
        }
        
        if( key == 'a' || displayPose == true)
        {
            readCameraConfig(cameraMatrix,distCoeffs);
            displayFeatures = false;
            displayPose = true;
            if(found)
            {
                cv::solvePnP(point_set, corner_set, cameraMatrix, distCoeffs, rvecs, tvecs);
                std::cout << "Rotation Matrix:" << std::endl << rvecs << std::endl << "Translation Matrix:" << std::endl << tvecs << std::endl;
                std::vector<cv::Point2f> imageAxisEndPoint;
                std::vector<cv::Point3f> worldAxisEndPoint;
                worldAxisEndPoint.push_back(cv::Point3f(square_side,0.0,0.0));
                worldAxisEndPoint.push_back(cv::Point3f(0.0,-square_side,0.0));
                worldAxisEndPoint.push_back(cv::Point3f(0.0,0.0,-square_side));
                
                cv::projectPoints(worldAxisEndPoint, rvecs, tvecs, cameraMatrix, distCoeffs, imageAxisEndPoint);
                cv::arrowedLine(frame,corner_set[22],imageAxisEndPoint[0],cv::Scalar(255,0,0), 5);
                cv::arrowedLine(frame,corner_set[22],imageAxisEndPoint[1],cv::Scalar(0,255,0), 5);
                cv::arrowedLine(frame,corner_set[22],imageAxisEndPoint[2],cv::Scalar(0,0,255), 5);
                
                //Display 3D Model
                std::vector<cv::Point2f> objPoints;
                cv::projectPoints(op.vertices, rvecs, tvecs, cameraMatrix, distCoeffs, objPoints);
                for(int i = 0; i<op.faceVertices.size(); i++)
                {
                    for(int j = 0; j<op.faceVertices[i].size() - 1; j++)
                    {
                        cv::line(frame,objPoints[op.faceVertices[i][j]-1],objPoints[op.faceVertices[i][j+1]-1],cv::Scalar(255,255,0), 1);
                    }
                }
                
            }
        }
        if(key == 'x')
        {
            displayPose = false;
            displayFeatures = false;
        }
        
        if( key == 'd' || displayFeatures == true)
        {
            displayPose = false;
            displayFeatures = true;
            std::vector<cv::Point2f> corners;
            cv::goodFeaturesToTrack(gray, corners, 100, 0.1, 10);

            for (int idx = 0; idx < corners.size(); idx++) {
                cv::circle(frame, corners.at(idx), 2, cv::Scalar(0, 0, 255), 2);
            }

            std::cout << corners.size() << std::endl;

        }
    
        if( key == 'q')
        {
            break;
        }
        
        cv::imshow("Video", frame);
        cv::imshow("Gray", gray);
    }

    delete capdev;
    return(0);
}
